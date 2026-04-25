/**
 * sample.h — 锁相环(DPLL) 头文件
 * 方法：同步采样 + FFT测相 + PID控制
 * 架构参考 01_signal_separator 验证过的方案
 */

#ifndef __SAMPLE_H
#define __SAMPLE_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "pid.h"

#define PLL_FFT_NUM      256        // FFT点数 = 单次采样点数
#define PLL_SAMPLE_RATE  256000.0f  // 采样率 256kHz

// ── 全局变量 ────────────────────────────────────
extern uint16_t g_adcIn[PLL_FFT_NUM];   // ADC2: 外部输入信号
extern uint16_t g_adcOut[PLL_FFT_NUM];  // ADC1: DDS输出反馈
extern float    g_ddsFreq;              // DDS当前输出频率 (Hz)
extern float    g_phaseView;            // 相位差 (度) 用于OLED显示

// 调试变量
extern float    g_debugVinMax;
extern float    g_debugVinMin;
extern float    g_debugVoutMax;
extern float    g_debugVoutMin;

// ── 函数声明 ────────────────────────────────────
void DPLL_Init(void);
void DPLL_Loop(void);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif /* __SAMPLE_H */
