#ifndef __MYFFT_H__
#define __MYFFT_H__

#include <stdint.h>
#include "arm_math.h"

// 波形类型
typedef enum {
    FFT_SINE = 0,
    FFT_SQUARE,
    FFT_TRIANGLE,
    FFT_UNKNOWN
} FFT_Wave_t;

// 窗函数
typedef enum {
    FFT_WIN_RECT = 0,    // 矩形窗（最快，相干采样时用）
    FFT_WIN_HANNING,     // 汉宁窗（默认，抗泄露）
} FFT_Window_t;

// FFT模式
typedef enum {
    FFT_MODE_FAST = 0,   // 快速: 256点+矩形窗, 测频/波形识别
    FFT_MODE_PRECISE,    // 精准: 1024点+汉宁窗, 精确幅值/相位
} FFT_Mode_t;

// FFT结果
typedef struct {
    float frequency;      // 基频
    float amplitude;      // 基频幅值(V)
    float phase;          // 基频相位(rad)
    float phase_diff;     // 双通道相位差(°)，-180~180
    FFT_Wave_t wave_type; // 波形类型
    // 双信号分离结果
    float freq2;          // 第二信号频率
    float amp2;           // 第二信号幅值(V)
    FFT_Wave_t wave2;     // 第二信号波形
    // 调试
    float process_ms;     // 处理耗时
} FFT_Result;

// FFT句柄（内部定义，外部用指针）
typedef struct FFT_Handle FFT_Handle;

// 初始化/反初始化
FFT_Handle* MYFFT_Init(float sample_rate, float adc_vref, uint8_t adc_bits);
void MYFFT_DeInit(FFT_Handle *h);

// 运行时配置
void MYFFT_SetMode(FFT_Handle *h, FFT_Mode_t mode);
void MYFFT_SetSampleRate(FFT_Handle *h, float fs);
void MYFFT_SetWindow(FFT_Handle *h, FFT_Window_t win);

// 单信号处理
void MYFFT_Process(FFT_Handle *h, const uint16_t *adc_buf, FFT_Result *result);

// 双信号分离
void MYFFT_ProcessDual(FFT_Handle *h, const uint16_t *adc_buf,
                        FFT_Result *r_low, FFT_Result *r_high);

// 双通道相位差（交错ADC数据: [ch1,ch2,ch1,ch2,...]，256点FFT）
void MYFFT_ProcessPhase(FFT_Handle *h, const uint16_t *dual_adc_buf, FFT_Result *result);

// 相位偏置校准（硬件固有延迟补偿，默认0）
void MYFFT_SetPhaseOffset(FFT_Handle *h, float offset_deg);

// 工具
// 获取内部幅度谱缓冲（只读，fft_len/2项）
const float* MYFFT_GetMagBuffer(FFT_Handle *h);

const char* FFT_WaveStr(FFT_Wave_t w);
const char* FFT_ModeStr(FFT_Mode_t m);

#endif
