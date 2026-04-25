/**
 * competition6 - 纯李萨如图显示
 *
 * 只保留：ADC1(PB1) + ADC3(PC0) 同时采样 → OLED画李萨如
 * 去掉：EdgeSync、编码器、按键、DAC
 *
 * 采样：TIM3 TRGO 1MHz 同时触发 ADC1 和 ADC3
 *   ADC1 → DMA1_Stream0 → adc1_buf[]（X轴，PB1/INP5）
 *   ADC3 → DMA1_Stream4 → adc3_buf[]（Y轴，PC0/INP0）
 *
 * 关键修复：
 *   1. 缓冲区从2048→256，1MHz采样下=0.256ms窗口≈25个100kHz周期
 *      2048点=2ms=200个周期叠加→模糊旋转
 *   2. 画一帧OLED≈23ms，DMA每0.128ms翻半区→必须等DMA回调再画
 *   3. 去掉EdgeSync中断(优先级0)→不再抢占I2C→不卡死
 */

#include "main.h"
#include "oled.h"
#include "mydraw.h"
#include <stdio.h>
#include <string.h>

// ── ADC缓冲 ──
// 256点@1MHz = 256µs窗口，100kHz信号≈25.6个周期
// 足够画清晰李萨如，又不会太多周期叠加
#define ADC_BUF_LEN  256
#define HALF_BUF    (ADC_BUF_LEN / 2)

__attribute__((aligned(32))) uint16_t adc1_buf[ADC_BUF_LEN];  // X轴
__attribute__((aligned(32))) uint16_t adc3_buf[ADC_BUF_LEN];  // Y轴

// DMA中断标志
volatile uint8_t adc1_half_done = 0;
volatile uint8_t adc1_full_done = 0;

// ── 初始化 ──
void main_init(void)
{
    OLED_Init();

    // ── ADC3手动初始化 ──
    MX_ADC3_Init();

    // ── 清ADC缓冲（DCache一致性） ──
    memset(adc1_buf, 0, sizeof(adc1_buf));
    memset(adc3_buf, 0, sizeof(adc3_buf));

    // ── ADC校准（H7必须做） ──
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    // ── 启动ADC DMA（先DMA再TIM） ──
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC_BUF_LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_buf, ADC_BUF_LEN);

    // ── 启动TIM3 1MHz采样触发 ──
    HAL_TIM_Base_Start(&htim3);

    // ── 停掉不需要的定时器 ──
    HAL_TIM_Base_Stop(&htim4);
    HAL_TIM_Base_Stop(&htim5);

    MYDRAW_Init();
    OLED_ShowString(1, 1, "LISSAJOUS");
}

// ── 主循环 ──
void main_loop(void)
{
    // 等DMA数据就绪再画
    uint8_t half_rdy = adc1_half_done;
    uint8_t full_rdy = adc1_full_done;

    if (half_rdy) {
        adc1_half_done = 0;
        // DCache Invalidate
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc1_buf, sizeof(uint16_t) * HALF_BUF);
        SCB_InvalidateDCache_by_Addr((uint32_t *)adc3_buf, sizeof(uint16_t) * HALF_BUF);
        MYDRAW_DrawLissajous(adc1_buf, adc3_buf, HALF_BUF);
    } else if (full_rdy) {
        adc1_full_done = 0;
        SCB_InvalidateDCache_by_Addr((uint32_t *)&adc1_buf[HALF_BUF], sizeof(uint16_t) * HALF_BUF);
        SCB_InvalidateDCache_by_Addr((uint32_t *)&adc3_buf[HALF_BUF], sizeof(uint16_t) * HALF_BUF);
        MYDRAW_DrawLissajous(&adc1_buf[HALF_BUF], &adc3_buf[HALF_BUF], HALF_BUF);
    }
}

// ── ADC DMA回调 ──
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        adc1_half_done = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        adc1_full_done = 1;
    }
}
