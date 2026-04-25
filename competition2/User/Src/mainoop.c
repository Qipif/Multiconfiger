/**
 * competition2 - 主文件
 * 100kHz正弦同频稳定显示
 * 双ADC时域锁相：ADC2(PA6)=输入, ADC1(PB1)=DDS反馈
 */

#include "main.h"
#include "ad9833.h"
#include "sample.h"
#include "oled.h"
#include <stdio.h>

// ── AD9833句柄 ─────────────────────────────────────
AD9833_Handler hds;

// ── OLED刷新 ──────────────────────────────────────
static uint32_t s_oled_t = 0;

static void oled_refresh(void)
{
    if (HAL_GetTick() - s_oled_t < 200) return;
    s_oled_t = HAL_GetTick();

    char buf[32];
    OLED_Clear();

    // 第0行：输入频率
    snprintf(buf, sizeof(buf), "Fin: %.0f Hz", g_measuredFreq);
    OLED_ShowString(0, 0, buf);

    // 第1行：DDS输出频率
    snprintf(buf, sizeof(buf), "Fout: %.0f Hz", g_ddsFreq);
    OLED_ShowString(0, 1, buf);

    // 第2行：相位差
    if (fabsf(g_measuredFreq - g_ddsFreq) < 10.0f) {
        snprintf(buf, sizeof(buf), "Phase: %.1f [LOCK]", g_measuredPhase);
    } else {
        snprintf(buf, sizeof(buf), "Phase: %.1f deg", g_measuredPhase);
    }
    OLED_ShowString(0, 2, buf);
}

// ── 初始化 ──────────────────────────────────────────
void main_init(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "Competition2");
    OLED_ShowString(0, 1, "Phase Locking...");

    // AD9833: SPI2, CS=PC0
    AD9833_Init(&hds, wave_sine, 100000, 0, &hspi2, GPIOC, GPIO_PIN_0);

    // 启动TIM3（TRGO=Update事件，触发ADC采样）
    HAL_TIM_Base_Start(&htim3);

    // sampleLoop()里会自动启动第一次采样
}

// ── 主循环 ──────────────────────────────────────────
void main_loop(void)
{
    sampleLoop();
    oled_refresh();
}
