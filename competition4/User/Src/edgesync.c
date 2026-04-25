/**
* edgesync.c — 硬件边沿同步器 + 数字移相 v3
*
* 升级路径：
*   v1: EXTI → dwt_delay忙等 → GPIO翻转
*       ❌ 中断阻塞 + 接近半周期双稳态失稳（140°抖动根因）
*   v2: EXTI → TIM2硬件比较 → GPIO翻转
*       ✅ ISR不阻塞，但EXTI vs TIM2竞争输出 → 接近0°/180°撕裂
*   v3: EXTI → TIM2硬件比较 → DAC输出
*       ✅ 防竞争：MIN_DELAY直出 + 先禁CC1再操作 + 清NVIC pending
*       ✅ 幅值可调 0~3.3V（12bit DAC）
*
* 防竞争核心逻辑：
*   当 delay <= MIN_DELAY(20cycles≈50ns) 时，直接在EXTI ISR里写DAC，
*   不走TIM2，彻底避免"EXTI和TIM2抢输出"的竞争条件。
*   正常延时时，EXTI ISR先禁CC1中断→清状态→再设新CCR1，
*   保证TIM2 ISR不会在操作过程中抢跑。
*
* 硬件连接：
*   PA6  ← 输入方波（EXTI双边沿中断）
*   PA4  → 输出方波（DAC1_CH1，幅值可调）
*   TIM2 ← 32位硬件定时器（输出比较延时，非阻塞）
*/

#include "edgesync.h"
#include "main.h"

static EdgeSync_Handle hsync;

// DAC写值+触发（ISR内联用）
static inline void dac_write(uint16_t val)
{
    DAC1->DHR12R1 = val;                   // 写12bit右对齐值
    DAC1->SWTRIGR = DAC_SWTRIGR_SWTRIG1;   // 软件触发转换
}

// ---- EXTI中断回调：PA6双边沿触发 ----
void EdgeSync_OnEdge(void)
{
    // 读输入状态
    uint8_t input_high = (GPIOA->IDR & GPIO_PIN_6) != 0;
    // 确定输出状态：>=180°时反转
    uint8_t output_high = hsync.invert ? !input_high : input_high;

    if (hsync.delay_cycles <= EDGE_MIN_DELAY) {
        // ====== 极小延时 / 零延时：直接输出 ======
        // 不走TIM2，彻底避免EXTI vs TIM2竞争
        // 先关掉可能pending的CC，防止残留TIM2 ISR抢跑
        TIM2->DIER &= ~TIM_DIER_CC1IE;     // 禁CC1中断
        TIM2->SR = ~TIM_SR_CC1IF;          // 清CC1标志
        NVIC_ClearPendingIRQ(TIM2_IRQn);   // 清NVIC残留pending
        hsync.has_pending = 0;
        // 直接写DAC
        dac_write(output_high ? hsync.dac_high : hsync.dac_low);
    } else {
        // ====== 正常延时：走TIM2输出比较 ======
        // 关键：先禁CC1，防止TIM2 ISR在操作过程中抢跑
        TIM2->DIER &= ~TIM_DIER_CC1IE;

        // 如果有pending的CC还没触发，先立即执行（防丢边沿）
        if (hsync.has_pending) {
            dac_write(hsync.pending_high ? hsync.dac_high : hsync.dac_low);
        }

        // 存输出电平，设TIM2比较值
        hsync.pending_high = output_high;
        hsync.has_pending = 1;
        TIM2->CCR1 = TIM2->CNT + hsync.delay_cycles;
        TIM2->SR = ~TIM_SR_CC1IF;          // 清可能残留的CC1标志
        TIM2->DIER |= TIM_DIER_CC1IE;      // 使能CC1中断
    }

    hsync.cnt++;
}

// ---- TIM2 CC1中断回调：比较匹配时写DAC ----
void EdgeSync_TIM2_CC_Handler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR = ~TIM_SR_CC1IF;
        TIM2->DIER &= ~TIM_DIER_CC1IE;     // 单次：关CC1中断
        hsync.has_pending = 0;

        dac_write(hsync.pending_high ? hsync.dac_high : hsync.dac_low);
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
    hsync.has_pending = 0;
    hsync.amplitude = 2.7f;
    hsync.dac_high = (uint16_t)(2.7f / 3.3f * 4095.0f + 0.5f);
    hsync.dac_low = 0;

    // ⚠️ 覆盖ADC2/DAC1的模拟配置
    GPIO_InitTypeDef gpio = {0};

    // PA6: EXTI双边沿中断
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PA4: DAC1_CH1输出（必须配成ANALOG模式）
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // 重配DAC1_CH1为软件触发模式（CubeMX默认配的是TIM4触发+DMA）
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;  // ⚠️ 必须DISABLE
    sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    dac_write(0);

    // TIM2：32位自由运行计数器，用于输出比较延时
    __HAL_RCC_TIM2_CLK_ENABLE();

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    hsync.tim2_clock = (pclk1 < SystemCoreClock) ? (pclk1 * 2) : pclk1;

    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CCMR1 = 0;
    TIM2->CCR1 = 0;
    TIM2->SR = 0;
    TIM2->DIER = 0;
    TIM2->CR1 = TIM_CR1_CEN;

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

// ---- 设置输出幅值 ----
void EdgeSync_SetAmp(float volt)
{
    if (volt < 0.0f) volt = 0.0f;
    if (volt > 3.3f) volt = 3.3f;
    hsync.amplitude = volt;
    hsync.dac_high = (uint16_t)(volt / 3.3f * 4095.0f + 0.5f);
}

float EdgeSync_GetAmp(void)
{
    return hsync.amplitude;
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
