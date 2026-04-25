/**
 * edgesync.c — 硬件边沿同步器 + 数字移相 v2
 *
 * 升级核心：
 *   v1: EXTI → dwt_delay忙等 → GPIO翻转
 *       ❌ 中断阻塞 + 接近半周期时双稳态失稳（140°抖动根因）
 *   v2: EXTI → 设TIM2->CCR1 → 返回 → TIM2硬件比较中断 → GPIO翻转
 *       ✅ ISR不阻塞 + 硬件精确定时 + 全相位范围无抖
 *
 * 硬件连接：
 *   PA6  ← 输入方波（EXTI双边沿中断）
 *   PA4  → 输出方波（GPIO推挽）
 *   TIM2 ← 32位硬件定时器（输出比较延时，非阻塞）
 */

#include "edgesync.h"
#include "main.h"

static EdgeSync_Handle hsync;

// ---- EXTI中断回调：PA6双边沿触发 ----
void EdgeSync_OnEdge(void)
{
    // 读输入状态
    uint8_t input_high = (GPIOA->IDR & GPIO_PIN_6) != 0;
    // 确定输出状态：>=180°时反转
    uint8_t output_high = hsync.invert ? !input_high : input_high;

    if (hsync.delay_cycles == 0) {
        // 零延时：直接翻转
        if (output_high)
            GPIOA->BSRR = GPIO_PIN_4;
        else
            GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
    } else {
        // 有延时：用TIM2输出比较，ISR立即返回（不忙等！）
        // 如果上一个CC还没触发，先立即执行（防丢边沿）
        if (TIM2->DIER & TIM_DIER_CC1IE) {
            if (hsync.pending_high)
                GPIOA->BSRR = GPIO_PIN_4;
            else
                GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
        }
        // 存输出电平，设TIM2比较值
        hsync.pending_high = output_high;
        TIM2->CCR1 = TIM2->CNT + hsync.delay_cycles;
        TIM2->SR = ~TIM_SR_CC1IF;       // 清pending
        TIM2->DIER |= TIM_DIER_CC1IE;   // 使能CC1中断
    }

    hsync.cnt++;
}

// ---- TIM2 CC1中断回调：比较匹配时翻转GPIO ----
void EdgeSync_TIM2_CC_Handler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR = ~TIM_SR_CC1IF;
        TIM2->DIER &= ~TIM_DIER_CC1IE;  // 单次：关CC1中断，等下次EXTI再开

        if (hsync.pending_high)
            GPIOA->BSRR = GPIO_PIN_4;
        else
            GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
    }
}

// ---- 初始化 ----
void EdgeSync_Init(void)
{
    hsync.cnt = 0;
    hsync.freq_filt = 0.0f;
    hsync.target_phase_deg = 0.0f;
    hsync.delay_cycles = 0;
    hsync.invert = 0;
    hsync.pending_high = 0;

    // ⚠️ 覆盖ADC2/DAC1的模拟配置
    GPIO_InitTypeDef gpio = {0};

    // PA6: EXTI双边沿中断
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PA4: GPIO推挽输出
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // TIM2：32位自由运行计数器，用于输出比较延时
    __HAL_RCC_TIM2_CLK_ENABLE();

    // TIM2时钟 = APB1 timer clock
    // H7: APB1 prescaler!=1时，timer clock = PCLK1 * 2
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    hsync.tim2_clock = (pclk1 < SystemCoreClock) ? (pclk1 * 2) : pclk1;

    TIM2->PSC = 0;            // 不分频
    TIM2->ARR = 0xFFFFFFFF;   // 32位最大
    TIM2->CCMR1 = 0;          // CC1冻结模式（软件控制翻转）
    TIM2->CCR1 = 0;
    TIM2->SR = 0;
    TIM2->DIER = 0;           // 初始不使能任何中断
    TIM2->CR1 = TIM_CR1_CEN;  // 启动计数器

    // NVIC
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

// ---- 计算延时 ----
static void EdgeSync_CalcDelay(void)
{
    if (hsync.freq_filt <= 0.0f) {
        hsync.delay_cycles = 0;
        hsync.invert = 0;
        return;
    }

    float effective_phase = hsync.target_phase_deg;
    uint8_t new_invert = 0;

    // 相位>=180°：反转+缩短延时
    if (effective_phase >= 180.0f) {
        effective_phase -= 180.0f;
        new_invert = 1;
    }

    // 周期（TIM2周期数）
    float period_cycles = (float)hsync.tim2_clock / hsync.freq_filt;
    hsync.delay_cycles = (uint32_t)(effective_phase / 360.0f * period_cycles);
    hsync.invert = new_invert;
}

// ---- 设置目标相位 ----
void EdgeSync_SetPhase(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 355.0f) deg = 355.0f;
    hsync.target_phase_deg = deg;
    EdgeSync_CalcDelay();
}

float EdgeSync_GetFreq(void)
{
    return hsync.freq_filt;
}

// ---- 用DWT测频（主循环100ms采样一次） ----
void EdgeSync_UpdateFreq(void)
{
    static uint32_t last_dwt = 0;
    static uint32_t last_cnt = 0;
    static uint32_t last_tick = 0;

    uint32_t now_tick = HAL_GetTick();
    if (now_tick - last_tick < 100) return;

    uint32_t now_dwt = DWT->CYCCNT;
    uint32_t now_cnt = hsync.cnt;

    uint32_t dcnt = now_cnt - last_cnt;
    uint32_t dt = now_dwt - last_dwt;

    if (dcnt >= 4 && dt > 0) {
        float cpu_freq = (float)SystemCoreClock;
        float freq_raw = (float)dcnt * 0.5f * cpu_freq / (float)dt;

        if (hsync.freq_filt <= 0.0f)
            hsync.freq_filt = freq_raw;
        else
            hsync.freq_filt = hsync.freq_filt * 0.95f + freq_raw * 0.05f;

        EdgeSync_CalcDelay();
    }

    last_dwt = now_dwt;
    last_cnt = now_cnt;
    last_tick = now_tick;
}
