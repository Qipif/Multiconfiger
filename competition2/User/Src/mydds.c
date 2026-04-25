/**
 ******************************************************************************
 * @file    mydds.c
 * @brief   高质量 DDS 波形发生器（适配锁相环，支持 100kHz+ 平滑输出）
 * @note    基于动态点数技术，保持 DAC 更新率恒定 5MHz，波形点数随频率自适应
 ******************************************************************************
 */

#include "mydds.h"
#include <math.h>
#include <string.h>

/* 正弦波表（256 点，12 位，中心 2048） */
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

/* 双缓冲区（放在 SRAM2 避免 Cache 问题） */
//static uint16_t dds_buf0[512] __attribute__((section(".sram2")));
//static uint16_t dds_buf1[512] __attribute__((section(".sram2")));

/* 内部辅助：获取定时器时钟 */
static uint32_t _get_timer_clk(TIM_HandleTypeDef *htim)
{
#ifdef HAL_RCC_GetTimerFreq
    return HAL_RCC_GetTimerFreq(htim);
#else
    if (htim->Instance == TIM1 || htim->Instance == TIM8  ||
        htim->Instance == TIM15 || htim->Instance == TIM16 ||
        htim->Instance == TIM17) {
        return HAL_RCC_GetPCLK2Freq() * 2;
    } else {
        return HAL_RCC_GetPCLK1Freq() * 2;
    }
#endif
}

//核心函数
static void _dds_update(MYDDS_HandleTypeDef *hdds)
{
    if (hdds->current_freq <= 0.0f) return;

    // 目标 DAC 更新率（可调，10~20 MHz）
    #define TARGET_DAC_RATE 10000000UL   // 10 MHz，稳定且平滑
    uint32_t wave_points = TARGET_DAC_RATE / hdds->current_freq;

    // 限制点数范围，确保波形质量和定时器可行
    if (wave_points < 10) wave_points = 10;      // 最高 1 MHz（若频率更高则点数不足）
    if (wave_points > 512) wave_points = 512;    // 最低约 19.5 kHz（512 点已很平滑）
    hdds->wave_table_len = wave_points;

    // 配置定时器产生精确的采样率
    uint32_t timer_clk = _get_timer_clk(hdds->htim);
    uint32_t sample_rate = hdds->current_freq * wave_points;
    uint32_t period = (uint32_t)((float)timer_clk / sample_rate + 0.5f);
    if (period < 2) period = 2;
    if (period > 65535) period = 65535;

    HAL_TIM_Base_Stop(hdds->htim);
    __HAL_TIM_SET_AUTORELOAD(hdds->htim, period - 1);
    __HAL_TIM_SET_COUNTER(hdds->htim, 0);

    // 相位步进
    uint32_t phase_step = (uint32_t)(hdds->current_freq * 4294967296.0f / sample_rate);

    // 幅值缩放
    float scale = hdds->current_amp / 1.65f;
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;

    // 双缓冲切换
   uint8_t inactive = 1 - hdds->active_buf;
   uint16_t *buf = (inactive == 0) ? hdds->buf0 : hdds->buf1;   // 改用句柄内的缓冲区
   uint32_t phase = hdds->phase_acc;

    // 生成波形（线性插值可进一步平滑，但当前点数已够）
    for (uint32_t i = 0; i < wave_points; i++) {
        int32_t val;
        switch (hdds->current_type) {
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
                // 三角波：利用高10位（0~1023），0~511上升，512~1023下降
                uint32_t tri_phase = (phase >> 22) & 0x3FF;  // 0~1023
                if (tri_phase < 512) {
                    val = tri_phase * 8;  // 0 → 0, 511 → 4088
                } else {
                    val = (1023 - tri_phase) * 8; // 512 → 4088, 1023 → 0
                }
                break;
            }
            case DDS_WAVE_SQUARE: {
                // 方波：相位最高位为0输出高电平，为1输出低电平
                val = (phase & 0x80000000) ? 0 : 4095;
                break;
            }
        }
        // 应用幅度缩放和直流偏置
        int32_t shifted = (int32_t)((val - 2048) * scale) + 2048;
        if (shifted > 4095) shifted = 4095;
        if (shifted < 0)    shifted = 0;
        buf[i] = (uint16_t)shifted;
        phase += phase_step;
    }
    hdds->phase_acc = phase;

    SCB_CleanDCache_by_Addr((uint32_t*)buf, wave_points * sizeof(uint16_t));

    HAL_DAC_Stop_DMA(hdds->hdac, hdds->dac_channel);
    HAL_DAC_Start_DMA(hdds->hdac, hdds->dac_channel,
                      (uint32_t*)buf, wave_points, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(hdds->htim);
    hdds->active_buf = inactive;
}

/*============================================================================
 * 公共 API
 *============================================================================*/

void MYDDS_Init(MYDDS_HandleTypeDef *hdds, TIM_HandleTypeDef *htim,
                DAC_HandleTypeDef *hdac, uint32_t dac_channel)
{
    memset(hdds, 0, sizeof(MYDDS_HandleTypeDef));
    hdds->htim = htim;
    hdds->hdac = hdac;
    hdds->dac_channel = dac_channel;
    hdds->wave_table_len = 256;   // 最大点数
    hdds->active_buf = 0;

    // 默认参数（不立即输出，等待 SetWaveform）
    hdds->current_type = DDS_WAVE_SINE;
    hdds->current_freq = 1000.0f;
    hdds->current_amp  = 1.0f;
    hdds->current_phase = 0.0f;
    hdds->phase_acc = 0;
}

void MYDDS_SetWaveform(MYDDS_HandleTypeDef *hdds, DDS_WaveType_t wave_type,
                       float freq_hz, float amp_vpp, float phase_deg)
{
    hdds->current_type  = wave_type;
    hdds->current_freq  = freq_hz;
    hdds->current_amp   = amp_vpp / 2.0f;   // Vpp 转 Vpeak
    hdds->current_phase = phase_deg;

    // 设置绝对相位
    float phase_norm = phase_deg / 360.0f;
    hdds->phase_acc = (uint32_t)(phase_norm * 4294967296.0f);

    _dds_update(hdds);
}

void MYDDS_UpdateParams(MYDDS_HandleTypeDef *hdds, float freq_hz,
                        float amp_vpp, float phase_deg)
{
    hdds->current_freq = freq_hz;
    hdds->current_amp  = amp_vpp / 2.0f;

    // 若指定了相位，则覆盖为绝对相位；否则保持连续
    if (phase_deg != 0.0f) {
        float phase_norm = phase_deg / 360.0f;
        hdds->phase_acc = (uint32_t)(phase_norm * 4294967296.0f);
    }
    // 注意：不改变波形类型

    _dds_update(hdds);
}

void MYDDS_Start(MYDDS_HandleTypeDef *hdds)
{
    _dds_update(hdds);   // 重新生成并启动
}

void MYDDS_Stop(MYDDS_HandleTypeDef *hdds)
{
    HAL_DAC_Stop_DMA(hdds->hdac, hdds->dac_channel);
    HAL_TIM_Base_Stop(hdds->htim);
}
