/*
 * @description: C题主程序 —— 信号变换与李萨如图显示装置（DDS方案）
 * @approach:   AD9833 DDS输出 + 硬件移相器 + 本地李萨如显示
 * @note:        基本要求(1)移相器用硬件全通滤波，发挥部分程控移相用DDS
 */
#include "main.h"
#include "oled.h"
#include "encoder.h"
#include "ad9833.h"
#include "xform.h"
#include <stdio.h>
#include <math.h>

// ── 双DDS对象 ──────────────────────────────────────────────────
//  dds1: 程控移相输出（Y轴，100kHz，相位0~180°可调）
//  dds2: 2分频输出（X轴B信号，50kHz）
static AD9833_Handle s_dds1;
static AD9833_Handle s_dds2;

// ── 系统状态 ──────────────────────────────────────────────────────
typedef enum { XSRC_A, XSRC_B } XSource;  // A=直通，B=2分频
static XSource  s_xsrc  = XSRC_A;
static float    s_phase = 0.0f;    // 当前移相角度（度）
static uint8_t  s_amp   = 1;       // 幅度选择（0=0.5, 1=1.0, 2=2.0）

// ── 初始化 ──────────────────────────────────────────────────────────
void main_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();
    OLED_ShowString(1, 1, "C-题 DDS方案");

    // AD9833初始化（MCLK=25MHz，根据实际晶振修改）
    AD9833_Init(&s_dds1, 25.0f);
    AD9833_Init(&s_dds2, 25.0f);

    // DDS1：100kHz，相位0°，正弦波
    AD9833_SetFrequency(&s_dds1, 100000.0f);
    AD9833_SetPhase(&s_dds1, 0.0f);
    AD9833_SetWaveform(&s_dds1, wave_sine);

    // DDS2：50kHz（2分频），正弦波
    AD9833_SetFrequency(&s_dds2, 50000.0f);
    AD9833_SetPhase(&s_dds2, 0.0f);
    AD9833_SetWaveform(&s_dds2, wave_sine);

    // 编码器初始化（调相位用）
    static ENC_Handle enc;
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target    = ENC_CTRL_PHASE;
    // 程控移相步进5°，范围0~180°，误差≤1°
    enc.phase_range = (ENC_ParamRange){5.0f, 5.0f, 1.0f, 0.0f, 180.0f};
    enc.cur_phase  = 0.0f;
    enc.cur_amp   = 1.0f;

    OLED_ShowString(2, 1, "DDS Ready");
    HAL_Delay(500);
}

// ── 主循环 ────────────────────────────────────────────────────────
void main_loop(void)
{
    static uint32_t t = 0;
    static float    last_phase = -1.0f;

    // 编码器更新
    ENC_Event_t evt = ENC_Update(&enc);

    // 旋转：更新DDS相位
    if (enc.rotated) {
        enc.rotated = 0;
        s_phase = enc.cur_phase;
        AD9833_SetPhase(&s_dds1, s_phase);
    }

    // 短按：切换X轴信号源（A=直通 / B=2分频）
    if (evt == ENC_EVT_CLICK) {
        s_xsrc = (s_xsrc == XSRC_A) ? XSRC_B : XSRC_A;
    }

    // 长按：切换幅度（0.5 / 1.0 / 2.0 Vpp，通过外部放大/衰减实现）
    if (evt == ENC_EVT_LONG_PRESS) {
        s_amp = (s_amp + 1) % 3;
        // 幅度控制通过外部运放增益实现，此处只记录状态
    }

    // OLED刷新（200ms）
    if (HAL_GetTick() - t < 200) return;
    t = HAL_GetTick();

    char buf[17];

    // 第1行：相位
    sprintf(buf, "PH:%6.1f", s_phase);
    OLED_ShowString(1, 1, buf);

    // 第2行：X轴源 + 幅度
    const char *xsrc_str = (s_xsrc == XSRC_A) ? "X:A(直通)" : "X:B(50k)";
    const char *amp_str  = (s_amp == 0) ? "0.5V" : (s_amp == 1) ? "1.0V" : "2.0V";
    sprintf(buf, "%s %s", xsrc_str, amp_str);
    OLED_ShowString(2, 1, buf);

    // 第3行：DDS状态
    sprintf(buf, "D1:%dkHz", (int)(100000.0f / 1000.0f));
    OLED_ShowString(3, 1, buf);
    sprintf(buf, "D2:%dkHz", (int)(50000.0f / 1000.0f));
    OLED_ShowString(3, 9, buf);
}
