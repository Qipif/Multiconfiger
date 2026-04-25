/**
 * competition2 - 主文件
 * 数字锁相环(DPLL)：FFT测相 + PID控制
 * DMA ISR只负责置标志，主循环做FFT/PID/DDS更新
 */

#include "main.h"
#include "ad9833.h"
#include "sample.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

// ── AD9833句柄 ─────────────────────────────────────
AD9833_Handler hds;

// ── OLED刷新 ──────────────────────────────────────
static uint32_t s_oled_t = 0;
static uint8_t  s_oledPage = 0;  // 0=主页面，1=调试页面

static void oled_refresh(void)
{
    if (HAL_GetTick() - s_oled_t < 200) return;
    s_oled_t = HAL_GetTick();

    char buf[32];

    if (s_oledPage == 0) {
        snprintf(buf, sizeof(buf), "F:%.0f Hz", g_ddsFreq);
        OLED_ShowString(1, 1, buf);

        snprintf(buf, sizeof(buf), "P:%.1f deg", g_phaseView);
        OLED_ShowString(2, 1, buf);

        snprintf(buf, sizeof(buf), "L:%lu CB:%lu",
                 g_pllLoopCnt, g_cbCnt);
        OLED_ShowString(3, 1, buf);
    } else {
        snprintf(buf, sizeof(buf), "Vin:%d~%d",
                 (int)g_debugVinMin, (int)g_debugVinMax);
        OLED_ShowString(1, 1, buf);

        snprintf(buf, sizeof(buf), "Vout:%d~%d",
                 (int)g_debugVoutMin, (int)g_debugVoutMax);
        OLED_ShowString(2, 1, buf);

        if (fabsf(g_debugVinMax - g_debugVoutMax) < 100 &&
            fabsf(g_debugVinMin - g_debugVoutMin) < 100) {
            OLED_ShowString(3, 1, "WARN:SAME!");
        } else {
            OLED_ShowString(3, 1, "DIFF OK   ");
        }
    }
}

void check_button(void)
{
    // PA0按键切换OLED页面（0=主页面, 1=调试页面）
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
        HAL_Delay(50);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
            s_oledPage = !s_oledPage;
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
        }
    }
}

// ── 初始化 ──────────────────────────────────────────
void main_init(void)
{
    OLED_Init();
    OLED_ShowString(1, 1, "Competition2");
    OLED_ShowString(2, 1, "DPLL Starting...");

    // AD9833初始化：SPI2, CS=PC0, 初始频率100kHz
    AD9833_Init(&hds, wave_sine, 100000, 0, &hspi2, GPIOC, GPIO_PIN_0);

    // 启动锁相环（配置TIM3=256kHz, 启动ADC DMA）
    DPLL_Init();

    OLED_ShowString(2, 1, "DPLL Running... ");
}

// ── 主循环 ──────────────────────────────────────────
void main_loop(void)
{
    DPLL_Loop();        // FFT + PID + DDS更新
    check_button();
    oled_refresh();
}
