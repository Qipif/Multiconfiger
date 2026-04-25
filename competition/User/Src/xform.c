/*
 * @description: C题核心逻辑 —— 信号变换与李萨如图显示装置
 * @approach:   DDS(AD9833)输出 + ADC测频/测相
 * @note:       双AD9833：ch1=程控移相输出，ch2=2分频(50kHz)输出
 */

#include "main.h"
#include "ad9833.h"
#include "encoder.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

// ── AD9833 对象（两片）──────────────────────────────────────────────
// 引脚分配（根据实际接线修改）：
//   ch1(移相输出): 独立FSYNC
//   ch2(2分频输出): 独立FSYNC
//   SCLK/SDATA 可共用
static AD9833_Handle s_dds1;  // 程控移相输出（X轴或Y轴）
static AD9833_Handle s_dds2;  // 2分频输出（50kHz）

// ── 系统状态 ─────────────────────────────────────────────────────────
typedef enum {
    XSRC_A,   // X轴用原始信号（直通）
    XSRC_B    // X轴用2分频信号（DDS2输出）
} XSource;

static XSource   s_xsrc   = XSRC_A;   // 当前X轴信号源
static float    s_freq_in = 100000.0f; // 输入频率（可ADC测频更新）
static uint8_t  s_locked  = 0;        // 是否测到输入频率

// ── 初始化 ───────────────────────────────────────────────────────────
void Xform_Init(void)
{
    OLED_Init();
    OLED_ShowString(1, 1, "Xform Init");

    // AD9833 初始化（MCLK=25MHz，根据实际晶振修改）
    AD9833_Init(&s_dds1, 25.0f);
    AD9833_Init(&s_dds2, 25.0f);

    // 上电默认输出：DDS1=100kHz同相，DDS2=50kHz
    AD9833_SetFrequency(&s_dds1, s_freq_in);
    AD9833_SetPhase(&s_dds1, 0.0f);
    AD9833_SetWaveform(&s_dds1, wave_sine);

    AD9833_SetFrequency(&s_dds2, s_freq_in / 2.0f);
    AD9833_SetPhase(&s_dds2, 0.0f);
    AD9833_SetWaveform(&s_dds2, wave_sine);

    // 编码器初始化（调相位用）
    static ENC_Handle enc;
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target      = ENC_CTRL_PHASE;
    enc.phase_range = (ENC_ParamRange){5.0f, 5.0f, 10.0f, 0.0f, 180.0f};
    enc.cur_phase    = 0.0f;
    enc.cur_amp     = 1.0f;  // amp不用，占位

    OLED_ShowString(2, 1, "DDS Ready");
    HAL_Delay(500);
}

// ── 主循环 ───────────────────────────────────────────────────────────
void Xform_Loop(void)
{
    static uint32_t t = 0;
    static float    cur_phase = 0.0f;

    // 编码器更新（调节相位）
    ENC_Event_t evt = ENC_Update(&enc);
    if (enc.rotated) {
        enc.rotated = 0;
        cur_phase = enc.cur_phase;
        AD9833_SetPhase(&s_dds1, cur_phase);
    }

    // 短按：切换X轴信号源（A=直通 / B=2分频）
    if (evt == ENC_EVT_CLICK) {
        s_xsrc = (s_xsrc == XSRC_A) ? XSRC_B : XSRC_A;
    }

    // OLED 刷新（200ms）
    if (HAL_GetTick() - t < 200) return;
    t = HAL_GetTick();

    char buf[17];
    sprintf(buf, "PH:%5.1f", cur_phase);
    OLED_ShowString(1, 1, buf);

    sprintf(buf, "X:%s", s_xsrc == XSRC_A ? "A(直通)" : "B(50k)");
    OLED_ShowString(2, 1, buf);

    // 显示锁定状态
    sprintf(buf, "[%s]", s_locked ? "LK" : "..");
    OLED_ShowString(1, 10, buf);
}

// ── 外部调用：ADC测频后更新输入频率 ─────────────────────────────
void Xform_UpdateFreq(float freq_hz)
{
    if (freq_hz < 1000.0f) return;
    s_freq_in = freq_hz;
    s_locked  = 1;

    // DDS1 跟随输入频率（移相输出）
    AD9833_SetFrequency(&s_dds1, s_freq_in);

    // DDS2 = 输入频率/2（2分频输出）
    AD9833_SetFrequency(&s_dds2, s_freq_in / 2.0f);
}
