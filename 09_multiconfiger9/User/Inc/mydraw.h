#ifndef __MYDRAW_H__
#define __MYDRAW_H__

#include "main.h"
#include "oled.h"
#include "myfft.h"

// 显示模式枚举
typedef enum {
    DISP_MODE_WAVE = 0,
    DISP_MODE_LISSA,
    DISP_MODE_FFT_INFO,
    DISP_MODE_SPECTRUM
} DrawMode;

// 初始化绘图模块
void MYDRAW_Init(void);

// 设置当前显示模式
void MYDRAW_SetMode(DrawMode mode);

// 获取当前显示模式
DrawMode MYDRAW_GetMode(void);

// 切换到下一个显示模式
void MYDRAW_NextMode(void);

// 绘制叠加波形（使用 ch0_data, ch1_data）
void MYDRAW_DrawWaveOverlay(const uint16_t *ch0, const uint16_t *ch1, uint16_t len);

// 绘制李萨如图（使用 ch0_data, ch1_data）
void MYDRAW_DrawLissajous(const uint16_t *ch0, const uint16_t *ch1, uint16_t len);

// 绘制 FFT 相位差信息（使用 MYFFT_Result 分别处理两个通道的数据）
void MYDRAW_DrawFFTInfo(MYFFT_Handle *fft, const uint16_t *ch0, const uint16_t *ch1,
                        uint16_t len, uint16_t fft_len, MYFFT_Result *result);

// 绘制频谱图（使用已有的 fft_result）
void MYDRAW_DrawSpectrum(const MYFFT_Result *result, uint16_t fft_len);

#endif
