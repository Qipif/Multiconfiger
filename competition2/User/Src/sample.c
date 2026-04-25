/**
 * sample.c — 数字锁相环(DPLL)  v3
 * 方法：同步采样 + FFT测相 + PID频率控制
 * 架构：ISR只置标志 → 主循环停/处理/重启
 *
 * 硬件连接：
 *   ADC2 (PA6) = 外部输入信号 100kHz
 *   ADC1 (PB1) = AD9833输出反馈
 */

#include "sample.h"
#include "ad9833.h"
#include "main.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

// ── 全局变量 ────────────────────────────────────────
uint16_t g_adcIn[PLL_FFT_NUM];
uint16_t g_adcOut[PLL_FFT_NUM];
float    g_ddsFreq   = 100000.0f;
float    g_phaseView = 0.0f;
float    g_debugVinMax  = 0.0f;
float    g_debugVinMin  = 4096.0f;
float    g_debugVoutMax = 0.0f;
float    g_debugVoutMin = 4096.0f;
uint32_t g_pllLoopCnt = 0;            // PLL循环计数

// ── 内部变量 ────────────────────────────────────────
static float    s_fftBuf[2 * PLL_FFT_NUM];
static float    s_fftMag[PLL_FFT_NUM];
static arm_cfft_instance_f32 s_cfftInst;
static pid_struct_t s_phasePid;

static volatile uint8_t s_dataReady = 0;

extern AD9833_Handler  hds;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

// ── 查找峰值 ────────────────────────────────────────
static uint16_t findPeak(const float *mag, uint16_t start, uint16_t end)
{
    uint16_t peak = start;
    float maxVal = mag[start];
    for (uint16_t i = start + 1; i <= end; i++) {
        if (mag[i] > maxVal) { maxVal = mag[i]; peak = i; }
    }
    return peak;
}

// ── DMA传输完成回调（仅置标志，不碰外设）─────────────
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        s_dataReady = 1;
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    (void)hadc;
}

// ── FFT处理，返回基频相位 ────────────────────────────
static float fftGetPhase(const uint16_t *buf, float *outFreq)
{
    float dc = 0.0f;
    for (int i = 0; i < PLL_FFT_NUM; i++) dc += (float)buf[i];
    dc /= PLL_FFT_NUM;

    for (int i = 0; i < PLL_FFT_NUM; i++) {
        s_fftBuf[2 * i]     = (float)buf[i] - dc;
        s_fftBuf[2 * i + 1] = 0.0f;
    }

    arm_cfft_f32(&s_cfftInst, s_fftBuf, 0, 1);
    arm_cmplx_mag_f32(s_fftBuf, s_fftMag, PLL_FFT_NUM);

    uint16_t peakIdx = findPeak(s_fftMag, 2, PLL_FFT_NUM / 2);
    if (outFreq) {
        *outFreq = (float)peakIdx * PLL_SAMPLE_RATE / PLL_FFT_NUM;
    }
    return atan2f(s_fftBuf[2 * peakIdx + 1], s_fftBuf[2 * peakIdx]);
}

// ── 初始化 ────────────────────────────────────────────
void DPLL_Init(void)
{
    arm_cfft_init_f32(&s_cfftInst, PLL_FFT_NUM);
    pid_init(&s_phasePid, 3.0f, 0.02f, 0.1f, 500.0f, 500.0f, 0.0f);

    // TIM3 = 256kHz
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 781 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 先启动ADC DMA，再启动TIM3触发
    s_dataReady = 0;
    g_pllLoopCnt = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
    HAL_TIM_Base_Start(&htim3);
}

// ── 主循环处理 ────────────────────────────────────────
void DPLL_Loop(void)
{
    if (!s_dataReady) return;
    s_dataReady = 0;

    // 1. 先停TIM3，阻止后续ADC触发（冻结数据源）
    HAL_TIM_Base_Stop(&htim3);

    // 2. 停ADC DMA
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    // 3. 清DCache
    uint32_t addr_in  = (uint32_t)g_adcIn  & ~(uint32_t)0x1F;
    uint32_t addr_out = (uint32_t)g_adcOut & ~(uint32_t)0x1F;
    uint32_t size = (PLL_FFT_NUM * sizeof(uint16_t) + 31) & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr_in,  size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr_out, size);

    // 4. FFT测相
    float freqIn, phaseIn = fftGetPhase(g_adcIn, &freqIn);
    float phaseOut = fftGetPhase(g_adcOut, NULL);

    // 5. 相位误差
    float phaseErr = phaseIn - phaseOut;
    while (phaseErr > M_PI)  phaseErr -= 2.0f * M_PI;
    while (phaseErr < -M_PI) phaseErr += 2.0f * M_PI;
    g_phaseView = phaseErr * 57.29578f;

    // 6. 频率控制
    if (g_pllLoopCnt == 0) {
        g_ddsFreq = freqIn;  // 首轮用FFT测频直接设定
        pid_reset(&s_phasePid);
    } else {
        float deltaFreq = pid_calc(&s_phasePid, 0.0f, phaseErr);
        g_ddsFreq -= deltaFreq;
    }

    if (g_ddsFreq > 150000.0f) g_ddsFreq = 150000.0f;
    if (g_ddsFreq < 50000.0f)  g_ddsFreq = 50000.0f;

    // 7. 更新AD9833（主循环中安全调用SPI）
    AD9833_SetFrequency(&hds, g_ddsFreq);

    // 8. 调试信息
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        if (g_adcIn[i]  > g_debugVinMax)  g_debugVinMax  = g_adcIn[i];
        if (g_adcIn[i]  < g_debugVinMin)  g_debugVinMin  = g_adcIn[i];
        if (g_adcOut[i] > g_debugVoutMax) g_debugVoutMax = g_adcOut[i];
        if (g_adcOut[i] < g_debugVoutMin) g_debugVoutMin = g_adcOut[i];
    }
    g_pllLoopCnt++;

    // 9. 重启：先ADC DMA，后TIM3
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);
}
