/**
* edgesync.c — 硬件边沿同步器 + 数字移相 v4
*
* v4 升级（解决按键不灵敏）：
*   - EXTI只响应RISING（减半中断：200k→100k/sec）
*   - 下降沿由TIM2 CC1级联输出：
*     EXTI→CCR1=上升沿延时 → CC1输出上升沿电平+CCR1+=半周期 → CC1输出下降沿电平
*   - 中断优先级从0降到2/1（给按键EXTI0留空间）
*
* v3 防竞争逻辑保留：
*   - delay<=MIN_DELAY直出不走TIM2
*   - EXTI ISR先禁CC1再操作
*   - 清NVIC pending防残留
*
* 中断链路（100kHz输入）：
*   EXTI RISING: 100k/sec（v3是200k/sec双边沿）
*   TIM2 CC1:   200k/sec（上升沿+下降沿各一次）
*   总计: 300k/sec（v3是400k/sec，减25%）
*
* 硬件连接：
*   PA6  ← 输入方波（EXTI仅上升沿中断）
*   PA4  → 输出方波（DAC1_CH1，2.7V固定幅值）
*   TIM2 ← 32位硬件定时器（输出比较延时）
*/

#include "edgesync.h"
#include "main.h"

static EdgeSync_Handle hsync;

static void EdgeSync_CalcDelay(void);  // 前向声明

// DAC写值+触发（ISR内联用）
static inline void dac_write(uint16_t val)
{
    DAC1->DHR12R1 = val;                   // 写12bit右对齐值
    DAC1->SWTRIGR = DAC_SWTRIGR_SWTRIG1;   // 软件触发转换
}

// ---- EXTI中断回调：PA6仅上升沿触发 ----
void EdgeSync_OnEdge(void)
{
    // 上升沿：输入必为高
    uint8_t output_high = hsync.invert ? 0 : 1;

    // 半周期（TIM2周期数）
    uint32_t half_period = 0;
    if (hsync.freq_filt > 0.0f) {
        float period = (float)hsync.tim2_clock / hsync.freq_filt;
        half_period = (uint32_t)(period * 0.5f);
    }

    if (hsync.delay_cycles <= EDGE_MIN_DELAY) {
        // ====== 极小延时 / 零延时：直接输出上升沿 ======
        TIM2->DIER &= ~TIM_DIER_CC1IE;     // 禁CC1中断
        TIM2->SR = ~TIM_SR_CC1IF;          // 清CC1标志
        NVIC_ClearPendingIRQ(TIM2_IRQn);   // 清NVIC残留pending
        hsync.pending_state = 0;           // 无级联等待

        dac_write(output_high ? hsync.dac_high : hsync.dac_low);

        // 安排下降沿：半周期后
        if (half_period > EDGE_MIN_DELAY) {
            hsync.pending_high = !output_high;  // 下降沿=反转电平
            hsync.pending_state = 1;            // 1=等下降沿CC
            TIM2->CCR1 = TIM2->CNT + half_period;
            TIM2->SR = ~TIM_SR_CC1IF;
            TIM2->DIER |= TIM_DIER_CC1IE;
        } else if (half_period > 0) {
            // 半周期也太短，直接输出
            dac_write((!output_high) ? hsync.dac_high : hsync.dac_low);
        }
    } else {
        // ====== 正常延时：走TIM2输出比较 ======
        TIM2->DIER &= ~TIM_DIER_CC1IE;

        // 如果有pending的CC还没触发，先立即执行（防丢边沿）
        if (hsync.pending_state == 1) {
            dac_write(hsync.pending_high ? hsync.dac_high : hsync.dac_low);
        } else if (hsync.pending_state == 2) {
            // 上一个上升沿的延时CC还没触发，立即执行
            dac_write(output_high ? hsync.dac_high : hsync.dac_low);
        }

        // 安排上升沿的延时输出
        hsync.pending_high = output_high;
        hsync.pending_state = 2;           // 2=等上升沿延时CC
        TIM2->CCR1 = TIM2->CNT + hsync.delay_cycles;
        TIM2->SR = ~TIM_SR_CC1IF;
        TIM2->DIER |= TIM_DIER_CC1IE;
    }

    hsync.cnt++;
}

// ---- TIM2 CC1中断回调：级联输出 ----
// pending_state:
//   0 = 无等待
//   1 = 等下降沿CC（上升沿已经直接输出了）
//   2 = 等上升沿延时CC（输出后还要安排下降沿）
void EdgeSync_TIM2_CC_Handler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR = ~TIM_SR_CC1IF;

        if (hsync.pending_state == 2) {
            // 上升沿延时CC触发 → 输出上升沿电平 + 安排下降沿
            dac_write(hsync.pending_high ? hsync.dac_high : hsync.dac_low);

            // 半周期后输出下降沿
            uint32_t half_period = 0;
            if (hsync.freq_filt > 0.0f) {
                float period = (float)hsync.tim2_clock / hsync.freq_filt;
                half_period = (uint32_t)(period * 0.5f);
            }

            if (half_period > EDGE_MIN_DELAY) {
                hsync.pending_high = !hsync.pending_high;  // 下降沿电平
                hsync.pending_state = 1;                   // 等下降沿CC
                TIM2->CCR1 += half_period;                 // 级联
                // CC1中断保持使能
            } else {
                // 半周期太短，直接输出下降沿，不级联
                dac_write((!hsync.pending_high) ? hsync.dac_high : hsync.dac_low);
                hsync.pending_state = 0;
                TIM2->DIER &= ~TIM_DIER_CC1IE;
            }
        } else if (hsync.pending_state == 1) {
            // 下降沿CC触发 → 输出下降沿电平
            dac_write(hsync.pending_high ? hsync.dac_high : hsync.dac_low);
            hsync.pending_state = 0;
            TIM2->DIER &= ~TIM_DIER_CC1IE;  // 单次：关CC1中断
        } else {
            // 异常：无等待但CC1触发了
            TIM2->DIER &= ~TIM_DIER_CC1IE;
        }
    }
}

// ---- 初始化 ----
void EdgeSync_Init(void)
{
    hsync.cnt = 0;
    hsync.freq_filt = 0.0f;
    hsync.target_phase_deg = 0.0f;
    hsync.phase_offset = 0.0f;
    hsync.delay_cycles = 0;
    hsync.invert = 0;
    hsync.pending_high = 0;
    hsync.pending_state = 0;
    hsync.amplitude = 2.7f;
    hsync.dac_high = (uint16_t)(2.7f / 3.3f * 4095.0f + 0.5f);
    hsync.dac_low = 0;

    // 覆盖ADC2/DAC1的模拟配置
    GPIO_InitTypeDef gpio = {0};

    // PA6: EXTI仅上升沿中断（v4：减半中断量）
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_IT_RISING;       // 只上升沿！
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PA4: DAC1_CH1输出（必须配成ANALOG模式）
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // 重配DAC1_CH1为软件触发模式
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;  // 必须DISABLE
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

    // NVIC：降低优先级，给按键EXTI0留空间
    // EXTI9_5(PA6边沿): 优先级2（比按键低，按键是0）
    // TIM2(CC比较): 优先级1（必须比EXTI9_5高，因为EXTI里设CCR1后TIM2要能抢占）
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

// ---- 计算延时（内部用） ----
static void EdgeSync_CalcDelay(void)
{
    if (hsync.freq_filt <= 0.0f) {
        hsync.delay_cycles = 0;
        hsync.invert = 0;
        return;
    }

    // 实际相位 = 显示相位 - 校准偏移
    float actual_phase = hsync.target_phase_deg - hsync.phase_offset;

    // 归一化到 0~360°
    while (actual_phase < 0.0f)    actual_phase += 360.0f;
    while (actual_phase >= 360.0f) actual_phase -= 360.0f;

    uint8_t new_invert = 0;

    // 相位>=180°：反转+缩短延时
    if (actual_phase >= 180.0f) {
        actual_phase -= 180.0f;
        new_invert = 1;
    }

    // 周期（TIM2周期数）
    float period_cycles = (float)hsync.tim2_clock / hsync.freq_filt;
    hsync.delay_cycles = (uint32_t)(actual_phase / 360.0f * period_cycles);
    hsync.invert = new_invert;
}

// ---- 设置目标相位（显示值） ----
void EdgeSync_SetPhase(float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 355.0f) deg = 355.0f;
    hsync.target_phase_deg = deg;
    EdgeSync_CalcDelay();
}

// ---- 校准：当前示波器显示180°，记下显示值作为偏移 ----
void EdgeSync_Calibrate(float current_display_phase)
{
    hsync.phase_offset = current_display_phase - 180.0f;
    EdgeSync_CalcDelay();
}

// ---- 获取当前显示相位 ----
float EdgeSync_GetPhase(void)
{
    return hsync.target_phase_deg;
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

    if (dcnt >= 2 && dt > 0) {
        // v4: cnt只计上升沿（不再是双边沿），所以不需要*0.5
        float cpu_freq = (float)SystemCoreClock;
        float freq_raw = (float)dcnt * cpu_freq / (float)dt;

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
