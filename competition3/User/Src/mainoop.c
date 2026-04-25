/**
 * competition3 - 硬件边沿同步器 + 数字移相
 *
 * ❌ 旧方案：ADC→DMA→施密特触发→DAC = 采样系统做时序问题 = 必抖
 * ✅ 新方案：EXTI双边沿中断→DWT精确延时→GPIO翻转 = 事件驱动 = 不抖
 *
 * 移相原理：
 *   相位0-180°：延时 phase/360*周期 后跟随输入
 *   相位180-360°：反转输出 + 延时 (phase-180)/360*周期
 *   延时永远 ≤ 半周期 → 不会丢边沿
 *
 * 硬件连接：
 *   PA6  ← 输入方波（EXTI双边沿中断）
 *   PA4  → 输出方波（GPIO推挽，VERY_HIGH速度）
 *   PE9/PE11 ← 编码器 (TIM1) — 旋转调相位，5°步进
 *   PA0  ← 按键 — 短按校准，长按无操作
 */

#include "main.h"
#include "oled.h"
#include "edgesync.h"
#include "encoder.h"
#include <stdio.h>

// ── 编码器 ─────────────────────────────────────────
static ENC_Handle enc;

// ── 初始化 ──────────────────────────────────────────
void main_init(void)
{
    // DWT计数器（测频+精确延时）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();
    OLED_ShowString(1, 1, "EDGE SYNC  ");

    // 边沿同步器初始化：PA6 EXTI双边沿 → PA4 GPIO输出
    // ⚠️ 必须在MX_ADC2_Init/MX_DAC1_Init之后调用
    // 因为那些函数会把PA6/PA4配成模拟模式，EdgeSync_Init会重新配成EXTI/GPIO
    EdgeSync_Init();

    // 不再需要TIM3/TIM4（ADC/DAC触发用），停掉省资源
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_Base_Stop(&htim4);

    // 编码器：调相位，5°步进，0-355°
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target = ENC_CTRL_PHASE;
    enc.phase_range = (ENC_ParamRange){5.0f, 5.0f, 5.0f, 0.0f, 355.0f};
    enc.cur_phase = 0.0f;

    OLED_ShowString(1, 1, "EDGE SYNC  ");
}

// ── 主循环 ──────────────────────────────────────────
void main_loop(void)
{
    // 编码器更新
    ENC_Event_t evt = ENC_Update(&enc);
    if (enc.rotated) {
        enc.rotated = 0;
        // 更新相位
        EdgeSync_SetPhase(enc.cur_phase);
    }

    // 短按：校准（相位归零）
    if (evt == ENC_EVT_CLICK) {
        enc.cur_phase = 0.0f;
        EdgeSync_SetPhase(0.0f);
    }

    // 测频更新（DWT计算 + 更新延时）
    EdgeSync_UpdateFreq();

    // OLED 200ms刷新
    static uint32_t t = 0;
    if (HAL_GetTick() - t < 200) return;
    t = HAL_GetTick();

    float freq = EdgeSync_GetFreq();

    char buf[17];
    sprintf(buf, "%s %.0fHz",
            freq > 0 ? "LK" : "..",
            freq);
    OLED_ShowString(1, 1, buf);

    sprintf(buf, "PH: %.0f         ",
            enc.cur_phase);
    OLED_ShowString(2, 1, buf);
}
