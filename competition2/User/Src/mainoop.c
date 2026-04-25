/**
 * competition2 - 100kHz正弦同频稳定显示
 * 方案：ADC2采样输入测频 → AD9833输出同频 → OLED显示
 */

#include "main.h"
#include "ad9833.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

// ── 外部句柄声明 ─────────────────────────────────────────
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi2;

// AD9833句柄
AD9833_Handler hds;

// ADC缓冲（TIM3触发，1MHz采样，放SRAM1并Cache对齐）
#define ADC_BUF_LEN  512
uint16_t adc2_buf[ADC_BUF_LEN] __attribute__((section(".SRAM1"), aligned(32)));

// 测频状态
static uint32_t meas_freq = 0;      // 测量到的频率(Hz)
static uint8_t  freq_ready = 0;     // 是否测到有效频率
static float    dds_freq   = 100000.0f; // DDS输出频率

// ────────────────────────────────────────────────────────────
// 初始化
// ────────────────────────────────────────────────────────────
void main_init(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Competition2");
    OLED_ShowString(2, 1, "APLL Locking...");

    // AD9833: 正弦波, 初始100kHz, 相位0
    AD9833_Init(&hds, wave_sine, 100000, 0, &hspi2, GPIOC, GPIO_PIN_0);

    // 启动ADC2 DMA（TIM3触发）
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);

    // 启动TIM3（触发ADC2采样）
    HAL_TIM_Base_Start(&htim3);
}

// ────────────────────────────────────────────────────────────
// 测频：过零检测（在main_loop调用，不在中断里）
// ────────────────────────────────────────────────────────────
static void measure_freq(void)
{
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 50) return;  // 50ms测一次
    last_tick = HAL_GetTick();

    // DCache Invalidate：确保读到DMA写入的最新数据
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, ADC_BUF_LEN * sizeof(uint16_t));

    // 过零检测：找上升沿过2048的点
    uint16_t zc_indices[16];
    uint8_t  zc_count = 0;

    for (uint16_t i = 1; i < ADC_BUF_LEN && zc_count < 16; i++) {
        uint16_t prev = adc2_buf[i - 1];
        uint16_t curr = adc2_buf[i];
        // 上升过零：前一点 < 2048，当前点 >= 2048
        if (prev < 2048 && curr >= 2048) {
            zc_indices[zc_count++] = i;
        }
    }

    if (zc_count >= 2) {
        // 计算平均周期（采样点数）
        uint32_t total_diff = 0;
        for (uint8_t i = 1; i < zc_count; i++) {
            total_diff += zc_indices[i] - zc_indices[i - 1];
        }
        float avg_samples_per_cycle = (float)total_diff / (zc_count - 1);

        // 采样率 = TIM3频率 = 200MHz/(TIM3.Period+1)
        // TIM3.Period = 199 → 采样率 = 1MHz
        float fs = 1000000.0f;
        meas_freq = (uint32_t)(fs / avg_samples_per_cycle);
        freq_ready = 1;
    }
}

// ────────────────────────────────────────────────────────────
// 频率锁定：调整AD9833输出
// ────────────────────────────────────────────────────────────
static void freq_lock(void)
{
    if (!freq_ready) return;

    // 简单比例控制
    float err = (float)meas_freq - dds_freq;

    // 偏差大于1Hz才调整
    if (err > 1.0f || err < -1.0f) {
        dds_freq += err * 0.3f;  // 收敛因子

        // 限制范围
        if (dds_freq < 0)       dds_freq = 0;
        if (dds_freq > 300000)  dds_freq = 300000;

        AD9833_SetFrequency(&hds, dds_freq);
    }
}

// ────────────────────────────────────────────────────────────
// OLED显示刷新
// ────────────────────────────────────────────────────────────
static void oled_update(void)
{
    static uint32_t last_t = 0;
    if (HAL_GetTick() - last_t < 200) return;  // 200ms刷新
    last_t = HAL_GetTick();

    OLED_ShowString(3, 1, "Freq In:      ");
    if (freq_ready) {
        OLED_ShowNum(3, 10, meas_freq, 6);
    } else {
        OLED_ShowString(3, 10, "---");
    }

    OLED_ShowString(4, 1, "Freq Out:");
    OLED_ShowNum(4, 10, (uint32_t)dds_freq, 6);
}

// ────────────────────────────────────────────────────────────
// 主循环
// ────────────────────────────────────────────────────────────
void main_loop(void)
{
    measure_freq();
    freq_lock();
    oled_update();
}
