#ifndef __EDGESYNC_H
#define __EDGESYNC_H

#include "main.h"
#include <stdint.h>

// 硬件边沿同步器 + 数字移相 v2
// 核心：EXTI双边沿中断 → TIM2输出比较 → GPIO翻转
//
// v1: dwt_delay忙等 → 阻塞中断 + 接近半周期双稳态失稳 → ❌
// v2: TIM2硬件比较 → ISR立即返回 + 硬件精确定时 + 全相位范围稳定 → ✅
//
// 相位0°：输出紧跟输入，零延迟
// 相位0-180°：TIM2比较延时 phase/360*周期 后跟随
// 相位180-355°：反转输出 + TIM2比较延时 (phase-180)/360*周期
// 测频100ms采样 + 低通0.95/0.05

typedef struct {
    // 测频（DWT + cnt）
    uint32_t cnt;              // 边沿总计数（上升+下降），中断里递增
    float    freq_filt;        // 低通滤波后频率

    // 移相
    float    target_phase_deg; // 目标相位 0-355°，5°步进
    uint32_t delay_cycles;     // 延时（TIM2周期数），ISR直接用
    uint8_t  invert;           // 1=相位>=180°，输出反转

    // TIM2输出比较
    uint8_t  pending_high;     // 待输出电平（CC中断时用）
    uint32_t tim2_clock;       // TIM2时钟频率（Hz）
} EdgeSync_Handle;

// 初始化：PA6 EXTI双边沿 + PA4 GPIO推挽 + TIM2硬件比较
// ⚠️ 必须在MX_ADC2_Init/MX_DAC1_Init之后调用
void EdgeSync_Init(void);

// EXTI边沿回调（在stm32h7xx_it.c中调用）
void EdgeSync_OnEdge(void);

// TIM2 CC中断回调（在stm32h7xx_it.c中调用）
void EdgeSync_TIM2_CC_Handler(void);

// 获取测到的频率（滤波后）
float EdgeSync_GetFreq(void);

// 更新测频（主循环调用，100ms一次）
void EdgeSync_UpdateFreq(void);

// 设置目标相位（度）
void EdgeSync_SetPhase(float deg);

#endif
