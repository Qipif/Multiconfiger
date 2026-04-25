#ifndef __EDGESYNC_H
#define __EDGESYNC_H

#include "main.h"
#include <stdint.h>

// 硬件边沿同步器 + 数字移相 v3
// 核心：EXTI双边沿中断 → TIM2输出比较 → DAC输出方波
//
// v1: dwt_delay忙等 → 阻塞中断 + 接近半周期双稳态失稳 → ❌
// v2: TIM2硬件比较 → GPIO翻转 → ISR不阻塞 → ✅
//     但EXTI vs TIM2竞争输出 → 接近0°/180°时波形撕裂 → ❌
// v3: TIM2硬件比较 → DAC输出 → 幅值可调 + 防竞争 → ✅✅
//
// 防竞争核心：
//   1. delay <= MIN_DELAY → 直接输出，不走TIM2（避开撞车区）
//   2. EXTI ISR里先禁CC1再操作（原子化，防TIM2 ISR抢跑）
//   3. 清NVIC pending（防残留TIM2 ISR在EXTI ISR后触发）
//
// 相位0°：输出紧跟输入，零延迟
// 相位0-180°：TIM2比较延时 phase/360*周期 后跟随
// 相位180-355°：反转输出 + TIM2比较延时 (phase-180)/360*周期
// 幅值固定2.7V：DAC12bit，高电平=amp对应的DAC值，低电平=0
// 测频100ms采样 + 低通0.95/0.05
//
// 校准：
//   用户旋编码器使示波器显示180°，长按PA0记录当前相位值
//   之后的实际相位 = 显示相位 - 校准偏移
//   校准偏移存在phase_offset中

// 最小延时阈值：低于此值直接输出，不走TIM2
// 20 cycles @ 400MHz ≈ 50ns，远小于5°相位对应的56 cycles
#define EDGE_MIN_DELAY  20

typedef struct {
    // 测频（DWT + cnt）
    uint32_t cnt;              // 边沿总计数（上升+下降），中断里递增
    float    freq_filt;        // 低通滤波后频率

    // 移相
    float    target_phase_deg; // 目标相位 0-355°，5°步进（显示值）
    float    phase_offset;     // 校准偏移：示波器180°时记下的显示值
                               // 实际相位 = target_phase_deg - phase_offset
    uint32_t delay_cycles;     // 延时（TIM2周期数），ISR直接用
    uint8_t  invert;           // 1=相位>=180°，输出反转

    // TIM2输出比较
    uint8_t  pending_high;     // 待输出电平（CC中断时用）
    uint8_t  has_pending;      // 1=有CC等待触发
    uint32_t tim2_clock;       // TIM2时钟频率（Hz）

    // 幅值控制
    float    amplitude;        // 输出幅值 0.0~3.3V
    uint16_t dac_high;         // 高电平DAC值（12bit: 0~4095）
    uint16_t dac_low;          // 低电平DAC值（固定0）
} EdgeSync_Handle;

// 初始化：PA6 EXTI双边沿 + PA4 DAC输出 + TIM2硬件比较
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
