#ifndef __AD9833_H
#define __AD9833_H

#include "main.h"

// AD9833 控制引脚（根据实际接线修改）
#define AD9833_SCLK_Port  GPIOA
#define AD9833_SCLK_Pin   GPIO_PIN_5   // SPI_SCK
#define AD9833_SDATA_Port GPIOA
#define AD9833_SDATA_Pin  GPIO_PIN_7   // SPI_MOSI
#define AD9833_FSYNC_Port GPIOA
#define AD9833_FSYNC_Pin  GPIO_PIN_4   // CS（片选）

// 波形类型
typedef enum {
    wave_sine   = 0,
    wave_triangle,
    wave_square
} WaveType;

// AD9833 句柄
typedef struct {
    float freq;        // 当前频率
    float freq_step;    // 频率分辨率（取决于MCLK）
    uint32_t freq_reg;  // 频率寄存器值
    uint16_t phase_reg; // 相位寄存器值
    WaveType wave;
} AD9833_Handle;

// 初始化
void AD9833_Init(AD9833_Handle *h, float mclk_mhz);
// 设置频率（Hz）
void AD9833_SetFrequency(AD9833_Handle *h, float freq_hz);
// 设置相位（度，0~360）
void AD9833_SetPhase(AD9833_Handle *h, float phase_deg);
// 设置波形
void AD9833_SetWaveform(AD9833_Handle *h, WaveType wave);

#endif
