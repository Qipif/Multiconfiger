#ifndef __EDGESYNC_H__
#define __EDGESYNC_H__

#include "main.h"
#include <stdint.h>

// 硬件边沿同步器 + 数字移相 v4
// 核心：EXTI仅上升沿中断 → TIM2输出比较级联 → DAC输出方波
//
// v4 vs v3:
//   - EXTI仅RISING（减半中断：200k→100k/sec@100kHz）
//   - TIM2 CC1级联：上升沿延时CC + 半周期后下降沿CC
//   - 中断优先级：EXTI=2, TIM2=1（给按键EXTI0留空间）
//   - cnt只计上升沿（测频不需要*0.5）
//
// 防竞争（v3保留）：
//   1. delay <= MIN_DELAY → 直接输出，不走TIM2
//   2. EXTI ISR里先禁CC1再操作
//   3. 清NVIC pending防残留

// 最小延时阈值：低于此值直接输出，不走TIM2
// 20 cycles @ 400MHz ≈ 50ns，远小于5°相位对应的56 cycles
#define EDGE_MIN_DELAY  20

typedef struct {
    // 测频（DWT + cnt）
    uint32_t cnt;              // 上升沿总计数（v4: 只计上升沿）
    float    freq_filt;        // 低通滤波后频率

    // 移相
    float    target_phase_deg; // 目标相位 0-355°，5°步进（显示值）
    float    phase_offset;     // 校准偏移：示波器180°时记下的显示值
                               // 实际相位 = target_phase_deg - phase_offset
    uint32_t delay_cycles;     // 延时（TIM2周期数），ISR直接用
    uint8_t  invert;           // 1=相位>=180°，输出反转

    // TIM2输出比较级联
    // pending_state: 0=无等待, 1=等下降沿CC, 2=等上升沿延时CC
    uint8_t  pending_state;
    uint8_t  pending_high;     // 待输出电平（CC中断时用）
    uint32_t tim2_clock;       // TIM2时钟频率（Hz）

    // 幅值控制
    float    amplitude;        // 输出幅值 0.0~3.3V
    uint16_t dac_high;         // 高电平DAC值（12bit: 0~4095）
    uint16_t dac_low;          // 低电平DAC值（固定0）
} EdgeSync_Handle;

// 初始化：PA6 EXTI仅上升沿 + PA4 DAC输出 + TIM2硬件比较
// 必须在MX_ADC2_Init/MX_DAC1_Init之后调用
void EdgeSync_Init(void);

// EXTI上升沿回调（在stm32h7xx_it.c中调用）
void EdgeSync_OnEdge(void);

// TIM2 CC中断回调（在stm32h7xx_it.c中调用）
void EdgeSync_TIM2_CC_Handler(void);

// 获取测到的频率（滤波后）
float EdgeSync_GetFreq(void);

// 更新测频（主循环调用，100ms一次）
void EdgeSync_UpdateFreq(void);

// 设置目标相位（度，显示值）——内部自动减去校准偏移
void EdgeSync_SetPhase(float deg);

// 校准：记当前相位为180°偏移
// 调用时机：示波器显示180°时，长按PA0
void EdgeSync_Calibrate(float current_display_phase);

// 获取当前显示相位
float EdgeSync_GetPhase(void);

// 设置输出幅值（0.0~3.3V）
void EdgeSync_SetAmp(float volt);

// 获取当前幅值
float EdgeSync_GetAmp(void);

#endif
