#ifndef __MYFFT_H__
#define __MYFFT_H__

#include <stdint.h>
#include "arm_math.h"

/* 波形类型枚举 */
typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_UNKNOWN
} WaveType;

/* 测量结果结构体 */
typedef struct {
    WaveType wave_type;      // 波形类型
    float frequency;         // 基频
    float amplitude;         // 基频幅值
    float phase;             // 基频相位 (弧度)
    float snr;               // 信噪比 (dB)（预留）
    float *mag;              // 幅度谱指针（正频率部分，长度 = fft_size/2）
    uint16_t mag_len;        // 幅度谱长度
} MYFFT_Result;

/* FFT 句柄（不透明指针） */
typedef struct MYFFT_Handle MYFFT_Handle;

/* 初始化与反初始化 */
MYFFT_Handle* MYFFT_Init(uint16_t fft_size, float sample_rate,
                         float adc_vref, uint8_t adc_bits);
void MYFFT_DeInit(MYFFT_Handle *handle);

/* 单信号处理 */
void MYFFT_Process(MYFFT_Handle *handle, const uint16_t *adc_buf, MYFFT_Result *result);

/* 双信号混合处理 (分离并识别两个波形) */
void MYFFT_ProcessDual(MYFFT_Handle *handle, const uint16_t *adc_buf,
                       MYFFT_Result *res_low, MYFFT_Result *res_high);

/* 辅助工具 */
const char* WaveType_ToString(WaveType type);

#endif
