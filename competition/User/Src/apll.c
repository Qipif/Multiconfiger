/**
 * apll.c — APLL v11 真正锁死版
 *
 * v10的问题：ZC只测频不修相位 → 本质是FLL → 相位必漂
 * v11核心改进：
 *   1. 亚采样零交叉插值 → spp精度从1点→0.01点
 *   2. NCO相位累加器输出 → 不依赖瞬时符号
 *   3. 锁定冻结机制 → 锁住后频率不再追噪声
 *   4. ★ ZC时相位纠偏 → 每次过零把phase掰回来 → 真锁相
 *
 * 数据流：
 *   ADC采输入 → 写ring → 亚采样ZC测频+测相位误差
 *   → spp低通 → freq_filt频率滤波 → 锁定冻结
 *   → NCO phase累加 → ZC纠偏 → 相位偏移 → 方波输出
 */

#include "apll.h"
#include <math.h>

#define ADC_MID  2048.0f
#define DAC_MID  2048
#define DAC_MAX  4095

// 方波输出电平
#define SQ_HIGH  DAC_MAX   // 4095 → ~3.3V
#define SQ_LOW   0         // 0    → ~0V

// 施密特触发电平（ADC原始值减中点后的阈值）
#define SQ_HYS   50.0f

// ---- 单样本处理 ----
uint16_t APLL_Step(APLL_Handle *h, uint16_t adc)
{
    // ===== 1. 写入环形缓冲 =====
    h->ring[h->widx] = adc;
    float cur = (float)adc - ADC_MID;

    // ===== 2. 亚采样零交叉检测 + 相位纠偏 =====
    //    线性插值求过零点的精确位置，精度远高于整点
    //    ★ v11：ZC时同时修相位，实现真正锁相
    if (h->last_sample < 0 && cur >= 0) {
        // 插值：frac = |last| / (|last| + cur)
        float frac = (-h->last_sample) / (cur - h->last_sample);
        // zc_pos = 上一个点 + frac
        float zc_pos = (float)h->widx - 1.0f + frac;

        float diff = zc_pos - h->last_zc_pos;
        if (diff < 0) diff += (float)APLL_RING_SIZE;

        if (diff > 1.0f && diff < (float)(APLL_RING_SIZE / 2)) {
            // spp强低通（0.95/0.05），稳住频率
            h->spp = 0.95f * h->spp + 0.05f * diff;

            // 锁定判定：连续多次周期稳定 → 锁定
            float err = fabsf(diff - h->spp);
            if (err < 0.02f)
                h->lock_cnt++;
            else
                h->lock_cnt = 0;

            if (h->lock_cnt > 20)
                h->locked = 1;
        }

        // ===== ★ 相位纠偏（v11核心） =====
        // ZC时phase应该=0，如果偏了就掰回来
        // 这是真正锁相的关键：有相位误差反馈
        float phase_err = h->phase;
        if (phase_err > 0.5f) phase_err -= 1.0f;

        // 限幅防炸（最多修0.1个周期 = 36°）
        if (phase_err > 0.1f)  phase_err = 0.1f;
        if (phase_err < -0.1f) phase_err = -0.1f;

        // 轻微纠偏：只修20%，不震荡
        h->phase -= 0.2f * phase_err;

        // wrap
        if (h->phase < 0.0f)    h->phase += 1.0f;
        if (h->phase >= 1.0f)   h->phase -= 1.0f;

        h->last_zc_pos = zc_pos;
        h->zc_found = 1;
    }
    h->last_sample = cur;

    // ===== 3. 计算频率 =====
    float freq = (h->spp > 1.0f) ? (h->fs / h->spp) : 100000.0f;

    // 锁定后冻结频率（核心抗抖）
    if (!h->locked)
        h->freq_filt = 0.9f * h->freq_filt + 0.1f * freq;

    float f_use = h->freq_filt;

    // ===== 4. NCO相位累加 =====
    //    phase_step = 频率/采样率，每步累加
    float phase_step = f_use / h->fs;
    h->phase += phase_step;
    if (h->phase >= 1.0f)
        h->phase -= 1.0f;

    // ===== 5. 相位偏移（用户调的移相） =====
    float phase_out = h->phase + h->target_phase_deg / 360.0f;
    if (phase_out >= 1.0f)
        phase_out -= 1.0f;

    // ===== 6. 方波输出 =====
    //    phase_out < 0.5 → 正半周(高)，否则负半周(低)
    uint16_t dac_val;
    if (phase_out < 0.5f) {
        dac_val = (uint16_t)(DAC_MID + (DAC_MID - 1) * h->amp_scale);
    } else {
        dac_val = (uint16_t)(DAC_MID - (DAC_MID - 1) * h->amp_scale);
    }

    // ===== 7. 推进写入指针 =====
    h->widx = (h->widx + 1) & APLL_RING_MASK;

    return dac_val;
}

// ---- API ----

void APLL_Init(APLL_Handle *h, float fs)
{
    h->fs = fs;
    h->target_phase_deg = 0;
    h->amp_scale = 1.0f;

    h->spp = fs / 100000.0f;  // 默认猜100kHz
    h->last_sample = 0;
    h->last_zc_pos = 0;
    h->zc_found = 0;

    h->freq_filt = 100000.0f;  // 初始猜100kHz
    h->phase = 0;

    h->locked = 0;
    h->lock_cnt = 0;

    h->last_out = -1;
    h->inherent_offset = 0;
    h->delay_f = 0;

    h->widx = 0;

    for (int i = 0; i < APLL_RING_SIZE; i++)
        h->ring[i] = (uint16_t)ADC_MID;

    for (int i = 0; i < APLL_DAC_BUF; i++)
        h->dac_buf[i] = DAC_MID;
}

void APLL_SetPhase(APLL_Handle *h, float deg, float amp)
{
    h->target_phase_deg = fmodf(deg, 360.0f);
    if (h->target_phase_deg < 0) h->target_phase_deg += 360.0f;
    h->amp_scale = amp;
}

void APLL_Process(APLL_Handle *h, uint16_t *adc_data,
                  uint16_t dac_offset, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        h->dac_buf[dac_offset + i] = APLL_Step(h, adc_data[i]);
    }
}

float APLL_GetFreq(APLL_Handle *h)
{
    return h->freq_filt;
}

void APLL_Calibrate(APLL_Handle *h)
{
    // 校准时重置NCO相位到0，记录当前phase为参考
    h->inherent_offset = h->phase;
    h->phase = 0;
    h->target_phase_deg = 0;
}
