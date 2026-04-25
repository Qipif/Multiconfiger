/**
 * sample.c — IQ数字锁相环 v5
 * ISR只置标志，主循环做全部停/启/处理
 */

#include "sample.h"
#include "ad9833.h"
#include "main.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define COS_TABLE_SIZE    1024
#define PHASE_SHIFT       22
#define ADC_MID           2048.0f
#define ADC_SCALE         (1.0f / 2048.0f)

#define IQ_ALPHA          0.01f
#define PI_KP             0.5f
#define PI_KI             0.05f
#define PI_I_MAX           500.0f
#define PI_OUT_MAX         500.0f
#define FREQ_MIN           80000.0f
#define FREQ_MAX           120000.0f

uint16_t g_adcIn[PLL_FFT_NUM];
uint16_t g_adcOut[PLL_FFT_NUM];
float    g_ddsFreq    = 100000.0f;
float    g_phaseView  = 0.0f;
float    g_debugVinMax  = 0.0f;
float    g_debugVinMin  = 4096.0f;
float    g_debugVoutMax = 0.0f;
float    g_debugVoutMin = 4096.0f;
uint32_t g_pllLoopCnt  = 0;
uint32_t g_cbCnt       = 0;  // 回调触发次数

static float    s_cosTable[COS_TABLE_SIZE];
static uint32_t s_phaseAcc   = 0;
static float    s_I_filt     = 0.0f;
static float    s_Q_filt     = 0.0f;
static float    s_PI_I       = 0.0f;

static volatile uint8_t s_dataReady = 0;

extern AD9833_Handler  hds;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

// ── DMA回调：只置标志，不碰任何外设 ──────────────
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        s_dataReady = 1;
        g_cbCnt++;
    }
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) { (void)hadc; }

// ── IQ混频 ─────────────────────────────────────
static inline void iq_mix(float sig)
{
    uint32_t idx = s_phaseAcc >> PHASE_SHIFT;
    float I = sig * s_cosTable[idx];
    float Q = sig * s_cosTable[(idx + 768) & 0x3FF];
    s_I_filt += IQ_ALPHA * (I - s_I_filt);
    s_Q_filt += IQ_ALPHA * (Q - s_Q_filt);
    uint32_t step = (uint32_t)((double)g_ddsFreq / PLL_SAMPLE_RATE * 4294967296.0);
    s_phaseAcc += step;
}

// ── 初始化 ────────────────────────────────────────
void DPLL_Init(void)
{
    for (int i = 0; i < COS_TABLE_SIZE; i++)
        s_cosTable[i] = cosf(2.0f * M_PI * i / COS_TABLE_SIZE);

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 781 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    s_phaseAcc = 0;
    s_I_filt   = 0.0f;
    s_Q_filt   = 0.0f;
    s_PI_I     = 0.0f;
    g_pllLoopCnt = 0;
    g_cbCnt      = 0;
    s_dataReady  = 0;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
    HAL_TIM_Base_Start(&htim3);
}

// ── 主循环 ────────────────────────────────────────
void DPLL_Loop(void)
{
    if (!s_dataReady) return;
    s_dataReady = 0;

    // 1. 停TIM3(无新ADC触发)，停DMA
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    // 2. 清DCache
    uint32_t mask = ~(uint32_t)0x1F;
    uint32_t sz   = (PLL_FFT_NUM * sizeof(uint16_t) + 31) & mask;
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)g_adcIn  & mask), sz);
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint32_t)g_adcOut & mask), sz);

    // 3. IQ混频: 处理256个输入采样点
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        float sig = ((float)g_adcIn[i] - ADC_MID) * ADC_SCALE;
        iq_mix(sig);
    }

    // 4. 相位误差
    float phaseErr = atan2f(s_Q_filt, s_I_filt);
    g_phaseView = phaseErr * 57.29578f;

    // 5. PI环路滤波
    s_PI_I += PI_KI * phaseErr;
    if (s_PI_I >  PI_I_MAX)   s_PI_I =  PI_I_MAX;
    if (s_PI_I < -PI_I_MAX)   s_PI_I = -PI_I_MAX;
    float freqCorr = PI_KP * phaseErr + s_PI_I;
    if (freqCorr >  PI_OUT_MAX)  freqCorr =  PI_OUT_MAX;
    if (freqCorr < -PI_OUT_MAX)  freqCorr = -PI_OUT_MAX;
    g_ddsFreq += freqCorr;
    if (g_ddsFreq > FREQ_MAX) g_ddsFreq = FREQ_MAX;
    if (g_ddsFreq < FREQ_MIN) g_ddsFreq = FREQ_MIN;

    // 6. 更新AD9833 (主循环中安全)
    AD9833_SetFrequency(&hds, g_ddsFreq);

    // 7. 调试
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        if (g_adcIn[i]  > g_debugVinMax)  g_debugVinMax  = g_adcIn[i];
        if (g_adcIn[i]  < g_debugVinMin)  g_debugVinMin  = g_adcIn[i];
        if (g_adcOut[i] > g_debugVoutMax) g_debugVoutMax = g_adcOut[i];
        if (g_adcOut[i] < g_debugVoutMin) g_debugVoutMin = g_adcOut[i];
    }
    g_pllLoopCnt++;

    // 8. 重启：先ADC后TIM3
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);
}
