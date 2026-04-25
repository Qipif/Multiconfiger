#include "mydds_dual.h"
#include "mydds.h"
#include <math.h>
#include <string.h>

/* 正弦波表（256点，12位，中心2048） */
static const uint16_t sine_table[256] = {
    2048, 2098, 2148, 2199, 2249, 2299, 2348, 2398,
    2447, 2497, 2545, 2594, 2642, 2690, 2738, 2785,
    2831, 2878, 2923, 2968, 3013, 3057, 3100, 3143,
    3185, 3227, 3267, 3307, 3347, 3385, 3423, 3459,
    3495, 3531, 3565, 3598, 3630, 3662, 3692, 3722,
    3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045,
    4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065,
    4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
    3750, 3722, 3692, 3662, 3630, 3598, 3565, 3531,
    3495, 3459, 3423, 3385, 3347, 3307, 3267, 3227,
    3185, 3143, 3100, 3057, 3013, 2968, 2923, 2878,
    2831, 2785, 2738, 2690, 2642, 2594, 2545, 2497,
    2447, 2398, 2348, 2299, 2249, 2199, 2148, 2098,
    2048, 1998, 1948, 1897, 1847, 1797, 1748, 1698,
    1649, 1599, 1551, 1502, 1454, 1406, 1358, 1311,
    1265, 1218, 1173, 1128, 1083, 1039,  996,  953,
     911,  869,  829,  789,  749,  711,  673,  637,
     601,  565,  531,  498,  466,  434,  404,  374,
     346,  319,  292,  267,  243,  220,  198,  177,
     157,  138,  121,  104,   89,   75,   62,   51,
      40,   31,   23,   16,   11,    7,    3,    2,
       1,    2,    3,    7,   11,   16,   23,   31,
      40,   51,   62,   75,   89,  104,  121,  138,
     157,  177,  198,  220,  243,  267,  292,  319,
     346,  374,  404,  434,  466,  498,  531,  565,
     601,  637,  673,  711,  749,  789,  829,  869,
     911,  953,  996, 1039, 1083, 1128, 1173, 1218,
    1265, 1311, 1358, 1406, 1454, 1502, 1551, 1599,
    1649, 1698, 1748, 1797, 1847, 1897, 1948, 1998
};

/* 获取定时器时钟 */
static uint32_t _get_timer_clk(TIM_HandleTypeDef *htim) {
#ifdef HAL_RCC_GetTimerFreq
    return HAL_RCC_GetTimerFreq(htim);
#else
    if (htim->Instance == TIM1 || htim->Instance == TIM8 ||
        htim->Instance == TIM15 || htim->Instance == TIM16 ||
        htim->Instance == TIM17) {
        return HAL_RCC_GetPCLK2Freq() * 2;
    } else {
        return HAL_RCC_GetPCLK1Freq() * 2;
    }
#endif
}

/* 计算单个采样点 */
static uint16_t _calc_sample(DDS_WaveType_t type, uint32_t phase, float amp_scale) {
    int32_t val;
    switch (type) {
        case DDS_WAVE_SINE:
        default: {
            uint32_t idx_int = (phase >> 24) & 0xFF;
            uint32_t idx_next = (idx_int + 1) & 0xFF;
            uint32_t frac = (phase >> 8) & 0xFFFF;
            int32_t v0 = sine_table[idx_int];
            int32_t v1 = sine_table[idx_next];
            val = v0 + ((v1 - v0) * (int32_t)frac) / 65536;
            break;
        }
        case DDS_WAVE_TRIANGLE: {
            uint32_t tri_phase = (phase >> 22) & 0x3FF;
            if (tri_phase < 512) val = tri_phase * 8;
            else val = (1023 - tri_phase) * 8;
            break;
        }
        case DDS_WAVE_SQUARE: {
            val = (phase & 0x80000000) ? 0 : 4095;
            break;
        }
    }
    int32_t shifted = (int32_t)((val - 2048) * amp_scale) + 2048;
    if (shifted > 4095) shifted = 4095;
    if (shifted < 0)    shifted = 0;
    return (uint16_t)shifted;
}

/* 两个独立的32字节对齐缓冲区 */
static uint16_t dac1_buf[512] __attribute__((aligned(32)));
static uint16_t dac2_buf[512] __attribute__((aligned(32)));

/* 核心更新函数 */
static void _dual_dds_update(MYDDS_DualHandle *hdds) {
    if (hdds->freq1 <= 0.0f && hdds->freq2 <= 0.0f) return;

    float max_freq = (hdds->freq1 > hdds->freq2) ? hdds->freq1 : hdds->freq2;
    if (max_freq < 1.0f) max_freq = 1.0f;

    #define TARGET_DAC_RATE 10000000UL
    uint32_t wave_points = TARGET_DAC_RATE / max_freq;
    if (wave_points < 10) wave_points = 10;
    if (wave_points > 512) wave_points = 512;
    hdds->wave_points = wave_points;

    uint32_t timer_clk = _get_timer_clk(hdds->htim);
    uint32_t sample_rate = max_freq * wave_points;
    uint32_t period = (uint32_t)((float)timer_clk / sample_rate + 0.5f);
    if (period < 2) period = 2;
    if (period > 65535) period = 65535;

    HAL_TIM_Base_Stop(hdds->htim);
    __HAL_TIM_SET_AUTORELOAD(hdds->htim, period - 1);
    __HAL_TIM_SET_COUNTER(hdds->htim, 0);

    hdds->phase_step1 = (uint32_t)(hdds->freq1 * 4294967296.0f / sample_rate);
    hdds->phase_step2 = (uint32_t)(hdds->freq2 * 4294967296.0f / sample_rate);

    float phase_norm1 = fmodf(hdds->phase1_deg, 360.0f) / 360.0f;
    if (phase_norm1 < 0) phase_norm1 += 1.0f;
    hdds->phase_acc1 = (uint32_t)(phase_norm1 * 4294967296.0f);

    float phase_norm2 = fmodf(hdds->phase2_deg, 360.0f) / 360.0f;
    if (phase_norm2 < 0) phase_norm2 += 1.0f;
    hdds->phase_acc2 = (uint32_t)(phase_norm2 * 4294967296.0f);

    float scale1 = hdds->amp1 / 1.65f;
    if (scale1 > 1.0f) scale1 = 1.0f;
    if (scale1 < 0.0f) scale1 = 0.0f;
    float scale2 = hdds->amp2 / 1.65f;
    if (scale2 > 1.0f) scale2 = 1.0f;
    if (scale2 < 0.0f) scale2 = 0.0f;

    uint16_t *buf1 = hdds->wave_buf1;
    uint16_t *buf2 = hdds->wave_buf2;
    uint32_t phase1 = hdds->phase_acc1;
    uint32_t phase2 = hdds->phase_acc2;

    for (uint32_t i = 0; i < wave_points; i++) {
        buf1[i] = _calc_sample(hdds->type1, phase1, scale1);
        buf2[i] = _calc_sample(hdds->type2, phase2, scale2);
        phase1 += hdds->phase_step1;
        phase2 += hdds->phase_step2;
    }

    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buf1, wave_points * sizeof(uint16_t));
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buf2, wave_points * sizeof(uint16_t));

    HAL_DAC_Stop_DMA(hdds->hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(hdds->hdac, DAC_CHANNEL_2);

    HAL_DAC_Start_DMA(hdds->hdac, DAC_CHANNEL_1, (uint32_t*)buf1, wave_points, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(hdds->hdac, DAC_CHANNEL_2, (uint32_t*)buf2, wave_points, DAC_ALIGN_12B_R);

    HAL_TIM_Base_Start(hdds->htim);
}

/*============================================================================
 * 公共 API
 *============================================================================*/

void MYDDS_DualInit(MYDDS_DualHandle *hdds, TIM_HandleTypeDef *htim,
                    DAC_HandleTypeDef *hdac, uint32_t ch1, uint32_t ch2) {
    memset(hdds, 0, sizeof(MYDDS_DualHandle));
    hdds->htim = htim;
    hdds->hdac = hdac;
    hdds->dac_ch1 = ch1;
    hdds->dac_ch2 = ch2;
    hdds->wave_buf1 = dac1_buf;
    hdds->wave_buf2 = dac2_buf;
    hdds->type1 = hdds->type2 = DDS_WAVE_SINE;
    hdds->freq1 = hdds->freq2 = 1000.0f;
    hdds->amp1 = hdds->amp2 = 0.5f;
    hdds->phase1_deg = hdds->phase2_deg = 0.0f;
}

void MYDDS_DualSetWaveform(MYDDS_DualHandle *hdds,
                           DDS_WaveType_t type1, float freq1, float amp1_vpp, float phase1_deg,
                           DDS_WaveType_t type2, float freq2, float amp2_vpp, float phase2_deg) {
    hdds->type1 = type1;
    hdds->freq1 = freq1;
    hdds->amp1 = amp1_vpp / 2.0f;
    hdds->phase1_deg = phase1_deg;

    hdds->type2 = type2;
    hdds->freq2 = freq2;
    hdds->amp2 = amp2_vpp / 2.0f;
    hdds->phase2_deg = phase2_deg;

    _dual_dds_update(hdds);
}

void MYDDS_DualUpdateParams(MYDDS_DualHandle *hdds,
                            float freq1, float amp1_vpp, float phase1_deg,
                            float freq2, float amp2_vpp, float phase2_deg) {
    hdds->freq1 = freq1;
    hdds->amp1 = amp1_vpp / 2.0f;
    if (phase1_deg != 0.0f) hdds->phase1_deg = phase1_deg;

    hdds->freq2 = freq2;
    hdds->amp2 = amp2_vpp / 2.0f;
    if (phase2_deg != 0.0f) hdds->phase2_deg = phase2_deg;

    _dual_dds_update(hdds);
}

void MYDDS_DualStart(MYDDS_DualHandle *hdds) {
    _dual_dds_update(hdds);
}

void MYDDS_DualStop(MYDDS_DualHandle *hdds) {
    HAL_DAC_Stop_DMA(hdds->hdac, hdds->dac_ch1);
    HAL_DAC_Stop_DMA(hdds->hdac, hdds->dac_ch2);
    HAL_TIM_Base_Stop(hdds->htim);
}
