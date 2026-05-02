/**
 * sample.h — FLL+PLL双环锁相 头文件
 * FLL(零交叉测频) + PLL(Costas相位锁定)
 */

#ifndef __SAMPLE_H
#define __SAMPLE_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"

#define PLL_FFT_NUM      256        // 采样点数 = DMA缓冲区大小
#define PLL_SAMPLE_RATE  256000.0f  // 采样率 256kHz

// ── 全局变量 ────────────────────────────────────
extern uint16_t g_adcIn[PLL_FFT_NUM];   // ADC2: 外部输入信号
extern uint16_t g_adcOut[PLL_FFT_NUM];  // ADC1: DDS输出反馈
extern float    g_ddsFreq;              // DDS当前输出频率 (Hz)
extern float    g_phaseView;            // 相位差 (度) 用于OLED显示
extern float    g_freqMeas;             // FLL测频结果 (Hz)
extern uint8_t  g_isLocked;             // 锁定标志

// 调试变量
extern float    g_debugVinMax;
extern float    g_debugVinMin;
extern float    g_debugVoutMax;
extern float    g_debugVoutMin;
extern uint32_t g_pllLoopCnt;    // PLL循环计数
extern uint32_t g_cbCnt;         // DMA回调次数

// ── 函数声明 ────────────────────────────────────
void DPLL_Init(void);
void DPLL_Loop(void);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* __SAMPLE_H */
