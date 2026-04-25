/**
 * sample.h - 时域锁相环头文件
 * 双ADC同步采样：ADC2(PA6)=输入, ADC1(PB1)=DDS输出反馈
 */

#ifndef __SAMPLE_H__
#define __SAMPLE_H__

#include "stm32h7xx_hal.h"

// ── 宏定义 ──────────────────────────────────────────────
#define SAMPLE_NUM       512
#define TARGET_FREQ      100000.0f   // 目标频率 100kHz
#define PID_KP           0.1f        // 比例系数
#define PID_KI           0.01f       // 积分系数
#define TIM3_TRIGGER_FREQ  1000000.0f  // TIM3触发频率=1MHz

// ── 锁相状态机 ──────────────────────────────────────────
typedef enum {
    PHASE_IDLE,       // 空闲
    PHASE_SAMPLING,   // 正在采样
    PHASE_PROC,       // 处理数据、调频
} PhaseState;

// ── 外部变量声明 ────────────────────────────────────────
extern PhaseState g_phaseState;
extern uint8_t    g_syncFlag;
extern float      g_measuredFreq;   // 测量到的输入频率(Hz)
extern float      g_measuredPhase;  // 相位差(度)
extern float      g_ddsFreq;        // DDS输出频率(Hz)，供OLED显示
extern uint16_t   g_adcIn[SAMPLE_NUM];   // ADC2: 输入信号(PA6)
extern uint16_t   g_adcOut[SAMPLE_NUM];  // ADC1: DDS输出反馈(PB1)

// ── 函数声明 ────────────────────────────────────────────
void sampleLoop(void);
void phaseLockStart(void);
void phaseLockStop(void);

// HAL回调函数（在stm32h7xx_it.c里调用）
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#endif
