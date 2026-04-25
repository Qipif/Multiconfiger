/*
 * @description: C题主程序 —— APLL延迟线锁相方案
 * @approach:   ADC采输入 → 环形缓冲+延迟插值 → DAC输出同频锁相信号
 * @key:        ADC和DAC同时钟(TIM2) → 延迟=绝对时间 → 相位天然锁死
 *
 * 硬件连接：
 *   ADC2(PA6) ← 输入100kHz正弦信号
 *   DAC1_CH1(PA4) → 输出同频锁相信号 → 外加RC低通(~150kHz)重建
 *   DAC1_CH2(PA5) → 备用/监视
 *
 * 采样率：1MHz (TIM2 ARR=63, 64MHz/64=1MHz)
 * 100kHz信号 → 每周期10个点 → APLL延迟线够用
 * DAC输出阶梯波 → 外部RC低通 → 平滑正弦波
 */

#include "main.h"
#include "oled.h"
#include "encoder.h"
#include "apll.h"
#include <stdio.h>
#include <math.h>

// ── APLL对象 ──────────────────────────────────────────────────
static APLL_Handle s_apll;

// ── DMA缓冲区 ─────────────────────────────────────────────────
// ADC2输入缓冲（PA6，TIM3触发，DMA CIRCULAR）
// 但APLL要求ADC和DAC同时钟！所以改为：ADC1单通道(PA6) + TIM2触发
// 暂时先用现有配置，后续用CubeMX改

// 暂用ADC1的DMA缓冲（单通道模式，只取PA6的数据）
// 缓冲区大小 = DAC缓冲区大小 / 2（Half/Full各一半）
#define ADC_BUF_LEN  APLL_DAC_BUF

// 外部变量（HAL生成的）
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// ADC输入缓冲（只取一路）
static uint16_t s_adcBuf[ADC_BUF_LEN];

// ── 系统状态 ──────────────────────────────────────────────────
static float    s_phase = 0.0f;    // 当前移相角度（度）
static float    s_amp   = 1.0f;    // 幅度缩放
static uint32_t s_cbCnt = 0;       // DMA回调计数

// ── OLED刷新 ──────────────────────────────────────────────────
static uint32_t s_oled_t = 0;
static uint8_t  s_oledPage = 0;  // 0=主页面，1=调试

static void oled_refresh(void)
{
    if (HAL_GetTick() - s_oled_t < 200) return;
    s_oled_t = HAL_GetTick();

    char buf[17];

    if (s_oledPage == 0) {
        // 主页面：频率 + 相位 + 锁定
        float freq = APLL_GetFreq(&s_apll);
        sprintf(buf, "F:%.0fHz", freq);
        OLED_ShowString(1, 1, buf);

        sprintf(buf, "PH:%.1f AM:%.1f", s_phase, s_amp);
        OLED_ShowString(2, 1, buf);

        sprintf(buf, "CB:%lu", s_cbCnt);
        OLED_ShowString(3, 1, buf);
    } else {
        // 调试页面：spp + delay + ring状态
        sprintf(buf, "spp:%.1f", s_apll.spp);
        OLED_ShowString(1, 1, buf);

        sprintf(buf, "dly:%.1f", s_apll.delay_f);
        OLED_ShowString(2, 1, buf);

        sprintf(buf, "CB:%lu zc:%d", s_cbCnt, s_apll.zc_found);
        OLED_ShowString(3, 1, buf);
    }
}

// ── DMA回调：ADC半帧/全帧完成 ─────────────────────────────────
// 处理ADC数据 → APLL延迟插值 → 填DAC缓冲

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    // 处理前半帧
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)((uint32_t)s_adcBuf & ~(uint32_t)0x1F),
        ((ADC_BUF_LEN / 2 * sizeof(uint16_t)) + 31) & ~(uint32_t)0x1F);

    APLL_Process(&s_apll, s_adcBuf, 0, ADC_BUF_LEN / 2);
    s_cbCnt++;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    // 处理后半帧
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)s_adcBuf + ADC_BUF_LEN/2 * sizeof(uint16_t)) & ~(uint32_t)0x1F),
        ((ADC_BUF_LEN / 2 * sizeof(uint16_t)) + 31) & ~(uint32_t)0x1F);

    APLL_Process(&s_apll, s_adcBuf + ADC_BUF_LEN / 2, ADC_BUF_LEN / 2, ADC_BUF_LEN / 2);
}

// ── 编码器 ────────────────────────────────────────────────────
static ENC_Handle s_enc;

// ── 按键回调（PA0，校准用）────────────────────────────────────
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin) {
        // 按下PA0 → 校准：当前延迟位置设为0°参考
        APLL_Calibrate(&s_apll);
    }
}

// ── 初始化 ──────────────────────────────────────────────────────
void main_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();
    OLED_ShowString(1, 1, "APLL 100kHz");
    OLED_ShowString(2, 1, "Initializing...");

    // ── 1. APLL初始化 ──
    // 采样率1MHz：TIM2 ARR=63, 64MHz/(63+1)=1MHz
    // ⚠️ 但CubeMX里ARR=199(320kHz)，这里软件改ARR
    APLL_Init(&s_apll, 1000000.0f);  // fs=1MHz

    // ── 2. 修改TIM2为1MHz ──
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 63);   // 64MHz / 64 = 1MHz
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // ── 3. 修改TIM4(DAC触发)也改为1MHz ──
    //    APLL核心：ADC和DAC必须同时钟！
    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 63);   // 64MHz / 64 = 1MHz
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // ── 4. 启动DAC DMA ──
    //    DAC1_CH1(PA4)输出APLL处理后的信号
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                       (uint32_t*)s_apll.dac_buf, APLL_DAC_BUF,
                       DAC_ALIGN_12B_R);

    // ── 5. 启动ADC DMA ──
    //    用ADC1单通道采PA6输入信号
    //    ⚠️ 当前ADC1是ScanMode(2通道)，CubeMX需要改成单通道
    //    暂时先启ADC1的DMA，只用一半数据
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_adcBuf, ADC_BUF_LEN);

    // ── 6. 启动定时器 ──
    HAL_TIM_Base_Start(&htim2);  // ADC触发
    HAL_TIM_Base_Start(&htim4);  // DAC触发

    // ── 7. 编码器 ──
    ENC_Init(&s_enc, &htim1, KEY_GPIO_Port, KEY_Pin);
    s_enc.target = ENC_CTRL_PHASE;
    s_enc.phase_range = (ENC_ParamRange){10.0f, 5.0f, 90.0f, 0.0f, 360.0f};
    s_enc.cur_phase = 0.0f;
    s_enc.cur_amp = 1.0f;

    OLED_ShowString(2, 1, "APLL Running  ");
}

// ── 主循环 ────────────────────────────────────────────────────────
void main_loop(void)
{
    // 编码器更新
    ENC_Event_t evt = ENC_Update(&s_enc);

    // 旋转：更新相位/幅度
    if (s_enc.rotated) {
        s_enc.rotated = 0;
        s_phase = s_enc.cur_phase;
        s_amp = s_enc.cur_amp;
        APLL_SetPhase(&s_apll, s_phase, s_amp);
    }

    // 短按：切换编码器控制目标（相位/幅度）
    if (evt == ENC_EVT_CLICK) {
        if (s_enc.target == ENC_CTRL_PHASE) {
            s_enc.target = ENC_CTRL_AMP;
        } else {
            s_enc.target = ENC_CTRL_PHASE;
        }
    }

    // 长按：校准
    if (evt == ENC_EVT_LONG_PRESS) {
        APLL_Calibrate(&s_apll);
    }

    // OLED刷新
    oled_refresh();
}
