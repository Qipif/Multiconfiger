/**
* competition5 - 硬件边沿同步器 + 数字移相 + 相位校准 + 预设
*
* v4: 按键改EXTI中断驱动+消抖+时间片，EXTI只RISING
*
* 模式1：编码器调实时相位，长按PA0=校准（示波器180°时按下）
* 模式2：编码器调预设值（第三行显示），短按PA0=写入预设值到相位
* 短按PA0：模式1↔2切换
* OLED右上角：1/2 表示当前模式
*
* 硬件连接：
*   PA6  ← 输入方波（EXTI仅上升沿中断）
*   PA4  → 输出方波（DAC1_CH1，2.7V固定幅值）
*   PE9/PE11 ← 编码器 (TIM1) — 5°步进
*   PA0  ← 按键 — EXTI0中断驱动，主循环5ms时间片消费
*/

#include "main.h"
#include "oled.h"
#include "edgesync.h"
#include "encoder.h"
#include <stdio.h>

// ── 模式 ─────────────────────────────────────────────
#define MODE_PHASE  1   // 调实时相位 + 长按校准
#define MODE_PRESET 2   // 调预设值 + 短按写入

// ── 编码器 ─────────────────────────────────────────
static ENC_Handle enc;

// ── 状态 ──────────────────────────────────────────────
static uint8_t  cur_mode   = MODE_PHASE;  // 当前模式
static float    preset_val = 0.0f;        // 预设相位值

// ── 初始化 ──────────────────────────────────────────
void main_init(void)
{
    // DWT计数器（测频）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();
    OLED_ShowString(1, 1, "EDGE SYNC  ");

    // 边沿同步器初始化
    EdgeSync_Init();
    EdgeSync_SetAmp(2.7f);

    // 停掉不需要的定时器
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_Base_Stop(&htim4);

    // 编码器：5°步进，0-355°
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target = ENC_CTRL_PHASE;
    enc.phase_range = (ENC_ParamRange){5.0f, 5.0f, 5.0f, 0.0f, 355.0f};
    enc.cur_phase = 0.0f;

    OLED_ShowString(1, 1, "EDGE SYNC 1");
}

// ── 主循环 ──────────────────────────────────────────
void main_loop(void)
{
    // 编码器+按键：5ms时间片（防被高频中断打碎）
    static uint32_t key_tick = 0;
    uint32_t now = HAL_GetTick();

    if (now - key_tick >= 5) {
        key_tick = now;

        ENC_Event_t evt = ENC_Update(&enc);

        // ── 旋转处理 ──
        if (enc.rotated) {
            enc.rotated = 0;
            if (cur_mode == MODE_PHASE) {
                EdgeSync_SetPhase(enc.cur_phase);
            } else {
                preset_val = enc.cur_phase;
            }
        }

        // ── 短按PA0：模式切换 ──
        if (evt == ENC_EVT_CLICK) {
            if (cur_mode == MODE_PHASE) {
                cur_mode = MODE_PRESET;
                enc.cur_phase = preset_val;
            } else {
                enc.cur_phase = preset_val;
                EdgeSync_SetPhase(preset_val);
                cur_mode = MODE_PHASE;
            }
        }

        // ── 长按PA0：校准（模式1下有效） ──
        if (evt == ENC_EVT_LONG_PRESS && cur_mode == MODE_PHASE) {
            EdgeSync_Calibrate(enc.cur_phase);
        }
    }

    // 测频更新
    EdgeSync_UpdateFreq();

    // OLED 200ms刷新
    static uint32_t t = 0;
    if (now - t < 200) return;
    t = now;

    float freq = EdgeSync_GetFreq();

    char buf[17];

    // 第一行：锁定状态 + 频率 + 右上角模式号
    sprintf(buf, "%s%.0fHz    %d",
            freq > 0 ? "LK:" : ".. ",
            freq,
            cur_mode);
    OLED_ShowString(1, 1, buf);

    // 第二行：当前相位
    if (cur_mode == MODE_PHASE) {
        sprintf(buf, "PH:%.0f       ",
                enc.cur_phase);
    } else {
        sprintf(buf, "PH:%.0f       ",
                EdgeSync_GetPhase());
    }
    OLED_ShowString(2, 1, buf);

    // 第三行：预设值（模式2可编辑）
    sprintf(buf, "PR:%.0f       ",
            preset_val);
    OLED_ShowString(3, 1, buf);
}
