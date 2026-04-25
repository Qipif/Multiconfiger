/**
 * apll.c — 数字延迟线锁相 (APLL) — 方波输出版
 *
 * ADC采输入 → 环形缓冲 + 零交叉测频 + 延迟插值 → 输出方波
 * 同时钟 → 延迟=绝对时间 → 相位天然锁死
 *
 * 关键改进：输出方波而非正弦波
 *   - 正弦波需要几十个点/周期才能看，100kHz需要几MHz DAC
 *   - 方波只需要判断"正半周/负半周"，2个点/周期就够
 *   - 外部RC/LC低通滤波器(~150kHz)把方波变回正弦波
 *
 * 200kHz采样率 → 100kHz信号每周期2点 → 方波输出完全OK
 */

#include "apll.h"
#include <math.h>

#define ADC_MID  2048
#define DAC_MID  2048
#define DAC_MAX  4095

// 方波输出电平
#define SQ_HIGH  DAC_MAX   // 4095 → ~3.3V
#define SQ_LOW   0         // 0    → ~0V

// ---- 单样本处理（灵魂） ----
uint16_t APLL_Step(APLL_Handle *h, uint16_t adc)
{
    // 1. 写入环形缓冲
    h->ring[h->widx] = adc;

    // 2. 零交叉测频（每个点都检测，100kHz@200kHz每周期才2点不能跳过）
    int16_t cur = (int16_t)adc - ADC_MID;

    if (h->last_sample < 0 && cur >= 0) {
        uint16_t now = h->widx;
        uint16_t diff = (now >= h->last_zc_widx) ?
                        (now - h->last_zc_widx) :
                        (APLL_RING_SIZE + now - h->last_zc_widx);

        if (diff > 1 && diff < APLL_RING_SIZE / 2) {
            // spp低通平滑
            h->spp = 0.9f * h->spp + 0.1f * (float)diff;
            h->zc_found = 1;
        }

        h->last_zc_widx = now;
    }
    h->last_sample = cur;

    // 3. 计算目标延迟（用校准偏移）
    if (h->spp > 1.0f) {
        h->delay_target = h->inherent_offset
                        + h->target_phase_deg / 360.0f * h->spp;
    }

    // 4. 平滑延迟（防跳变）
    h->delay_f += h->delay_alpha * (h->delay_target - h->delay_f);

    // 5. 插值读取（从ring里读延迟后的值，只取符号）
    float ridx = (float)h->widx - h->delay_f;
    if (ridx < 0) ridx += (float)APLL_RING_SIZE;

    int i0 = (int)ridx & APLL_RING_MASK;
    int i1 = (i0 + 1) & APLL_RING_MASK;
    float frac = ridx - (float)(int)ridx;

    float v0 = (float)h->ring[i0] - ADC_MID;
    float v1 = (float)h->ring[i1] - ADC_MID;
    float v = v0 * (1.0f - frac) + v1 * frac;

    // 6. 方波输出：只需判断延迟后信号的符号
    //    v > 0 → 正半周 → 高电平
    //    v <= 0 → 负半周 → 低电平
    //    幅度控制：通过占空比微调或直接用DAC输出不同电平
    uint16_t dac_val;
    if (v > 0) {
        // 正半周：幅度控制
        // amp_scale=1.0 → 输出DAC_MAX, amp_scale=0.5 → 输出DAC_MAX*0.5
        dac_val = (uint16_t)(DAC_MID + (DAC_MID - 1) * h->amp_scale);
    } else {
        // 负半周
        dac_val = (uint16_t)(DAC_MID - (DAC_MID - 1) * h->amp_scale);
    }

    // 7. 推进写入指针
    h->widx = (h->widx + 1) & APLL_RING_MASK;

    return dac_val;
}

// ---- API ----

void APLL_Init(APLL_Handle *h, float fs)
{
    h->fs = fs;
    h->target_phase_deg = 0;
    h->amp_scale = 1.0f;

    h->spp = fs / 100000.0f;  // 默认猜100kHz → 200kHz/100kHz=2点
    h->last_sample = 0;
    h->last_zc_widx = 0;
    h->zc_found = 0;

    h->delay_f = 0;
    h->delay_target = 0;
    h->delay_alpha = 0.05f;
    h->inherent_offset = 0;

    h->widx = 0;

    for (int i = 0; i < APLL_RING_SIZE; i++)
        h->ring[i] = ADC_MID;

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
    if (h->spp > 1.0f)
        return h->fs / h->spp;
    return 0.0f;
}

void APLL_Calibrate(APLL_Handle *h)
{
    h->inherent_offset = h->delay_f;
}
