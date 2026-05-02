/**
 * competition - C题 APLL方波锁相方案
 *
 * 核心思路：
 *   ADC2(PA6)采100kHz正弦输入 → APLL延迟线 → DAC1(PA4)输出同频方波
 *   外部RC/LC低通(~150kHz)把方波变回正弦波 → 实现100kHz锁相
 *
 * 硬件连接：
 *   PA6  ← 输入100kHz正弦信号 (ADC2_CH3)
 *   PA4  → 输出100kHz方波 → 外接低通滤波 → 正弦波 (DAC1_CH1)
 *   PE9/PE11 ← 编码器 (TIM1)
 *   PA0  ← 按键（短按校准，长按切相位/幅度）
 *
 * 数据流：
 *   TIM3(200kHz) → ADC2采样 → DMA CIRCULAR → HalfCplt/FullCplt回调
 *   → APLL_Process(环形缓冲+延迟+方波输出) → dac_buf
 *   → DMA CIRCULAR → TIM4(200kHz) → DAC1输出
 */

#include "main.h"
#include "oled.h"
#include "apll.h"
#include "encoder.h"
#include <stdio.h>

// ── APLL + 编码器 ──────────────────────────────────────────
static APLL_Handle hapll;
static ENC_Handle enc;

// ── ADC缓冲区 ──────────────────────────────────────────────
#define ADC_BUF_LEN  APLL_DAC_BUF  // 256，和DAC缓冲区一样大
static uint16_t adc2_buf[ADC_BUF_LEN];

// ── 状态 ───────────────────────────────────────────────────
static volatile float measured_freq = 0.0f;
static volatile uint8_t apll_running = 0;

// ── 设置定时器频率（参考12_multiconfiger12）──────────────────
static void TIM_Set_Frequency(TIM_HandleTypeDef *htim, uint32_t freq_hz)
{
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SET_AUTORELOAD(htim, timer_clk / freq_hz - 1);
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
}

// ── ADC2 DMA回调 ───────────────────────────────────────────

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2 || !apll_running) return;

    uint16_t half = ADC_BUF_LEN / 2;

    // 处理前半帧 → 填DAC前半帧
    APLL_Process(&hapll, adc2_buf, 0, half);

    // DCache清理（让DMA能读到最新DAC数据）
    SCB_CleanDCache_by_Addr((uint32_t*)hapll.dac_buf, APLL_DAC_BUF * sizeof(uint16_t));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2 || !apll_running) return;

    uint16_t half = ADC_BUF_LEN / 2;

    // 处理后半帧 → 填DAC后半帧
    APLL_Process(&hapll, adc2_buf + half, half, half);

    // DCache清理
    SCB_CleanDCache_by_Addr((uint32_t*)hapll.dac_buf, APLL_DAC_BUF * sizeof(uint16_t));
}

// ── 初始化 ──────────────────────────────────────────────────
void main_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();

    // APLL初始化：200kHz采样率（100kHz信号每周期2点，方波够用）
    APLL_Init(&hapll, 200000.0f);

    // TIM3驱动ADC2, TIM4驱动DAC, 都设200kHz
    // 同时钟 → 延迟=绝对时间 → 相位锁死
    TIM_Set_Frequency(&htim3, 200000);
    TIM_Set_Frequency(&htim4, 200000);

    // 启动ADC2 DMA CIRCULAR（PA6输入信号）
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);

    // 启动DAC1 CH1 DMA CIRCULAR（PA4输出方波）
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t*)hapll.dac_buf, APLL_DAC_BUF,
                      DAC_ALIGN_12B_R);

    // 编码器：调相位/幅度
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target = ENC_CTRL_PHASE;
    enc.phase_range = (ENC_ParamRange){10.0f, 10.0f, 30.0f, 0.0f, 350.0f};
    enc.amp_range   = (ENC_ParamRange){0.1f, 0.1f, 0.2f, 0.0f, 2.0f};
    enc.cur_phase = 0.0f;
    enc.cur_amp = 1.0f;

    apll_running = 1;
    OLED_ShowString(1, 1, "APLL SQ 100k");
}

// ── 主循环 ──────────────────────────────────────────────────
void main_loop(void)
{
    // 编码器更新
    ENC_Event_t evt = ENC_Update(&enc);
    if (enc.rotated) {
        enc.rotated = 0;
        APLL_SetPhase(&hapll, enc.cur_phase, enc.cur_amp);
    }

    // 短按校准：当前延迟位置=0°
    if (evt == ENC_EVT_CLICK) {
        APLL_Calibrate(&hapll);
    }

    // 长按切换：相位↔幅度
    if (evt == ENC_EVT_LONG_PRESS) {
        if (enc.target == ENC_CTRL_PHASE)
            enc.target = ENC_CTRL_AMP;
        else
            enc.target = ENC_CTRL_PHASE;
    }

    // OLED 200ms刷新
    static uint32_t t = 0;
    if (HAL_GetTick() - t < 200) return;
    t = HAL_GetTick();

    measured_freq = APLL_GetFreq(&hapll);

    char buf[17];
    sprintf(buf, "%s %.0fHz",
            hapll.zc_found ? "LK" : "..",
            measured_freq);
    OLED_ShowString(1, 1, buf);

    // 第2行：相位+幅度
    if (enc.target == ENC_CTRL_PHASE) {
        sprintf(buf, "PH%.0f AM%.1f", hapll.target_phase_deg, enc.cur_amp);
    } else {
        sprintf(buf, "PH%.0f AM%.1f*", hapll.target_phase_deg, enc.cur_amp);
    }
    OLED_ShowString(2, 1, buf);
}
