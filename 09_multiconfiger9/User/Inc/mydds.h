#ifndef __MYDDS_H__
#define __MYDDS_H__

#include "main.h"
#include "tim.h"
#include "dac.h"

#define MYDDS_TABLE_SIZE 512   // 最大波形点数

typedef enum {
    DDS_WAVE_SINE,
    DDS_WAVE_TRIANGLE,
    DDS_WAVE_SQUARE
} DDS_WaveType_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    DAC_HandleTypeDef *hdac;
    uint32_t dac_channel;
    uint32_t wave_table_len;
    volatile uint8_t active_buf;

    DDS_WaveType_t current_type;
    float current_freq;
    float current_amp;      // Vpeak
    float current_phase;
    uint32_t phase_acc;

    // 为每个实例分配独立的双缓冲区
    uint16_t buf0[512];
    uint16_t buf1[512];
} MYDDS_HandleTypeDef;

void MYDDS_Init(MYDDS_HandleTypeDef *hdds, TIM_HandleTypeDef *htim,
                DAC_HandleTypeDef *hdac, uint32_t dac_channel);
void MYDDS_SetWaveform(MYDDS_HandleTypeDef *hdds, DDS_WaveType_t wave_type,
                       float freq_hz, float amp_vpp, float phase_deg);
void MYDDS_UpdateParams(MYDDS_HandleTypeDef *hdds, float freq_hz,
                        float amp_vpp, float phase_deg);
void MYDDS_Start(MYDDS_HandleTypeDef *hdds);
void MYDDS_Stop(MYDDS_HandleTypeDef *hdds);

#endif
