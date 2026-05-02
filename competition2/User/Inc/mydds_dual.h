#ifndef __MYDDS_DUAL_H__
#define __MYDDS_DUAL_H__

#include "main.h"
#include "tim.h"
#include "dac.h"
#include "mydds.h"

typedef struct {
    TIM_HandleTypeDef *htim;        // 共用定时器
    DAC_HandleTypeDef *hdac;        // DAC 外设
    uint32_t dac_ch1;               // DAC_CHANNEL_1
    uint32_t dac_ch2;               // DAC_CHANNEL_2

    // 通道 1 参数
    DDS_WaveType_t type1;
    float freq1;
    float amp1;                     // Vpeak
    float phase1_deg;

    // 通道 2 参数
    DDS_WaveType_t type2;
    float freq2;
    float amp2;
    float phase2_deg;

    // 内部状态
    uint32_t phase_acc1;
    uint32_t phase_acc2;
    uint32_t phase_step1;
    uint32_t phase_step2;
    uint32_t wave_points;           // 每通道点数
    uint16_t *wave_buf1;            // 通道1独立缓冲区
    uint16_t *wave_buf2;            // 通道2独立缓冲区
} MYDDS_DualHandle;

void MYDDS_DualInit(MYDDS_DualHandle *hdds, TIM_HandleTypeDef *htim,
                    DAC_HandleTypeDef *hdac, uint32_t ch1, uint32_t ch2);
void MYDDS_DualSetWaveform(MYDDS_DualHandle *hdds,
                           DDS_WaveType_t type1, float freq1, float amp1_vpp, float phase1_deg,
                           DDS_WaveType_t type2, float freq2, float amp2_vpp, float phase2_deg);
void MYDDS_DualUpdateParams(MYDDS_DualHandle *hdds,
                            float freq1, float amp1_vpp, float phase1_deg,
                            float freq2, float amp2_vpp, float phase2_deg);
void MYDDS_DualStart(MYDDS_DualHandle *hdds);
void MYDDS_DualStop(MYDDS_DualHandle *hdds);

#endif
