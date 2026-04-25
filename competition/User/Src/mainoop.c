#include "main.h"
#include "oled.h"
#include "apll.h"
#include "encoder.h"
#include <stdio.h>

// APLL: 数字延迟线锁相
// ADC采样→ring缓冲→延迟N点(浮点插值)→DAC输出
// 同时钟 → 天然锁相

#define ADC_BUF_LEN  2048

// 设为1=直通测试(ADC→DAC旁路APLL)，设0=正常APLL模式
#define BYPASS_TEST  1

static APLL_Handle hapll;
static ENC_Handle enc;

static uint16_t adc2_buf[ADC_BUF_LEN];
static volatile float measured_freq = 0.0f;
static volatile uint8_t apll_running = 0;

// 设置定时器频率
static void TIM_Set_Frequency(TIM_HandleTypeDef *htim, uint32_t freq_hz)
{
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SET_AUTORELOAD(htim, timer_clk / freq_hz - 1);
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
}

void main_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();

    // APLL初始化：1MHz采样率
    APLL_Init(&hapll, 1000000.0f);

    // TIM3驱动ADC2, TIM4驱动DAC, 都设1MHz
    TIM_Set_Frequency(&htim3, 1000000);
    TIM_Set_Frequency(&htim4, 1000000);

    // 启动ADC2 DMA CIRCULAR
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_BUF_LEN);

    // 启动DAC1 CH1 DMA CIRCULAR
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t*)hapll.dac_buf, APLL_DAC_BUF,
                      DAC_ALIGN_12B_R);

    // 编码器：调相位/幅度，长按切换目标
    ENC_Init(&enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    enc.target = ENC_CTRL_PHASE;
    enc.phase_range = (ENC_ParamRange){10.0f, 10.0f, 30.0f, 0.0f, 350.0f};
    enc.amp_range   = (ENC_ParamRange){0.1f, 0.1f, 0.2f, 0.0f, 2.0f};
    enc.cur_phase = 0.0f;
    enc.cur_amp = 1.0f;

    // OLED先显示，再启用APLL处理（避免回调风暴阻塞I2C）
    OLED_ShowString(1, 1, "APLL");
    apll_running = 1;
}

void main_loop(void)
{
    // 编码器更新（每次循环都跑）
    ENC_Event_t evt = ENC_Update(&enc);
    if (enc.rotated) {
        enc.rotated = 0;
        APLL_SetPhase(&hapll, enc.cur_phase, enc.cur_amp);
    }
    // 短按校准：当前延迟位置=0°
    // 长按切换：相位↔幅度
    if (evt == ENC_EVT_CLICK) {
        APLL_Calibrate(&hapll);
    }
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
    sprintf(buf, "%s %.1fHz",
            hapll.zc_found ? "LK" : "..",
            measured_freq);
    OLED_ShowString(1, 1, buf);

    // 第2行：当前目标+值
    if (enc.target == ENC_CTRL_PHASE) {
        sprintf(buf, "PH %.0f A%.1f", hapll.target_phase_deg, enc.cur_amp);
    } else {
        sprintf(buf, "PH %.0f AM %.1f", hapll.target_phase_deg, enc.cur_amp);
    }
    OLED_ShowString(2, 1, buf);
}

// ---- ADC2 DMA回调 ----

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2 || !apll_running) return;

    uint16_t half = ADC_BUF_LEN / 2;

    // DCache invalidation: DMA写了前半帧，CPU要读到真实数据
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, half * sizeof(uint16_t));

#if BYPASS_TEST
    // 直通：ADC数据直接复制到DAC，跳过APLL
    for (uint16_t i = 0; i < half; i++) {
        hapll.dac_buf[i] = adc2_buf[i];
    }
#else
    // 处理前半帧 → 填DAC前半帧
    APLL_Process(&hapll, adc2_buf, 0, half);
#endif

    // DCache清理: 让DMA把DAC数据搬走
    SCB_CleanDCache_by_Addr((uint32_t*)hapll.dac_buf, APLL_DAC_BUF * sizeof(uint16_t));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC2 || !apll_running) return;

    uint16_t half = ADC_BUF_LEN / 2;

    // DCache invalidation: DMA写了后半帧，CPU要读到真实数据
    SCB_InvalidateDCache_by_Addr((uint32_t*)(adc2_buf + half), half * sizeof(uint16_t));

#if BYPASS_TEST
    // 直通：ADC数据直接复制到DAC，跳过APLL
    for (uint16_t i = 0; i < half; i++) {
        hapll.dac_buf[half + i] = adc2_buf[half + i];
    }
#else
    // 处理后半帧 → 填DAC后半帧
    APLL_Process(&hapll, adc2_buf + half, half, half);
#endif

    // DCache清理: 让DMA把DAC数据搬走
    SCB_CleanDCache_by_Addr((uint32_t*)hapll.dac_buf, APLL_DAC_BUF * sizeof(uint16_t));
}
