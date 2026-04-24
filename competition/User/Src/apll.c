#include "apll.h"
#include <math.h>

#define ADC_MID  2048
#define DAC_MID  2048
#define DAC_MAX  4095

// ---- 单样本处理（灵魂） ----
uint16_t APLL_Step(APLL_Handle *h, uint16_t adc)
{
    // 1. 写入环形缓冲
    h->ring[h->widx] = adc;

    // 2. 零交叉测频（轻量在线检测）
    int16_t cur = (int16_t)adc - ADC_MID;

    if (h->last_sample < 0 && cur >= 0) {
        uint16_t now = h->widx;
        uint16_t diff = (now >= h->last_zc_widx) ?
                        (now - h->last_zc_widx) :
                        (APLL_RING_SIZE + now - h->last_zc_widx);

        if (diff > 10 && diff < APLL_RING_SIZE / 2) {
            h->spp = 0.9f * h->spp + 0.1f * (float)diff;
            h->zc_found = 1;
        }

        h->last_zc_widx = now;
    }
    h->last_sample = cur;

    // 3. 计算目标延迟（用校准偏移）
    //    inherent_offset: 校准时记录的delay_f，对应用户0°
    //    用户设phase_deg → delay = inherent_offset + phase_deg/360*spp
    if (h->spp > 1.0f) {
        h->delay_target = h->inherent_offset
                        + h->target_phase_deg / 360.0f * h->spp;
    }

    // 4. 平滑延迟（防跳变）
    h->delay_f += h->delay_alpha * (h->delay_target - h->delay_f);

    // 5. 插值读取
    float ridx = (float)h->widx - h->delay_f;
    if (ridx < 0) ridx += (float)APLL_RING_SIZE;

    int i0 = (int)ridx & APLL_RING_MASK;
    int i1 = (i0 + 1) & APLL_RING_MASK;
    float frac = ridx - (float)(int)ridx;

    float v0 = (float)h->ring[i0] - ADC_MID;
    float v1 = (float)h->ring[i1] - ADC_MID;
    float v = v0 * (1.0f - frac) + v1 * frac;

    // 幅度缩放
    v = v * h->amp_scale + DAC_MID;

    if (v < 0) v = 0;
    if (v > DAC_MAX) v = DAC_MAX;

    // 6. 推进写入指针
    h->widx = (h->widx + 1) & APLL_RING_MASK;

    return (uint16_t)v;
}

// ---- API ----

void APLL_Init(APLL_Handle *h, float fs)
{
    h->fs = fs;
    h->target_phase_deg = 0;
    h->amp_scale = 1.0f;

    h->spp = fs / 1000.0f;  // 默认猜1kHz
    h->last_sample = 0;
    h->last_zc_widx = 0;
    h->zc_found = 0;

    h->delay_f = 0;
    h->delay_target = 0;
    h->delay_alpha = 0.05f;  // 平滑系数，小一点更稳
    h->inherent_offset = 0;  // 校准前为0

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
    // 当前delay_f位置就是用户认为的0°
    h->inherent_offset = h->delay_f;
}
