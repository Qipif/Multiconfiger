/**
 * sample.c — 数字锁相环(DPLL)
 * 方法：同步采样 + FFT测相 + PID频率控制
 * 架构：ISR停TIM3冻结缓冲区 → 主循环FFT/PID → 更新DDS → 重启
 *
 * 硬件连接：
 *   ADC2 (PA6) = 外部输入信号 100kHz
 *   ADC1 (PB1) = AD9833输出反馈
 *   两路由TIM3同步触发，同步采样
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
uint16_t g_adcIn[PLL_FFT_NUM];      // ADC2: 外部输入
uint16_t g_adcOut[PLL_FFT_NUM];     // ADC1: DDS反馈
float    g_ddsFreq   = 100000.0f;    // DDS当前频率
float    g_phaseView = 0.0f;         // 相位差(度)
float    g_debugVinMax  = 0.0f;
float    g_debugVinMin  = 4096.0f;
float    g_debugVoutMax = 0.0f;
float    g_debugVoutMin = 4096.0f;

// ── 内部变量 ────────────────────────────────────────
static float    s_fftBuf[2 * PLL_FFT_NUM];   // FFT缓冲 (实虚交替)
static float    s_fftMag[PLL_FFT_NUM];       // 幅度谱
static arm_cfft_instance_f32 s_cfftInst;     // FFT实例
static pid_struct_t s_phasePid;              // PID控制器

static volatile uint8_t s_dataReady = 0;    // 0=等待 1=数据就绪
static uint8_t  s_firstRun = 1;             // 首次运行标志
static uint32_t s_loopCnt = 0;

extern AD9833_Handler  hds;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

// ── 查找幅度谱峰值 (跳过DC和低频噪声) ──────────────
static uint16_t findPeak(const float *mag, uint16_t start, uint16_t end)
{
    uint16_t peak = start;
    float maxVal = mag[start];
    for (uint16_t i = start + 1; i <= end; i++) {
        if (mag[i] > maxVal) {
            maxVal = mag[i];
            peak = i;
        }
    }
    return peak;
}

// ── DMA传输完成回调 ─────────────────────────────────
//    动作：①停TIM3(冻结ADC触发) ②清DCache ③置标志
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    // 关键：先停TIM3，阻止后续ADC触发，冻结缓冲区
    htim3.Instance->CR1 &= ~TIM_CR1_CEN;

    // H7关键：Invalidate DCache
    uint32_t addr_in  = (uint32_t)g_adcIn  & ~(uint32_t)0x1F;
    uint32_t addr_out = (uint32_t)g_adcOut & ~(uint32_t)0x1F;
    uint32_t size = (PLL_FFT_NUM * sizeof(uint16_t) + 31) & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr_in,  size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr_out, size);

    s_dataReady = 1;
}

// ── 不使用半完成回调 ─────────────────────────────────
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    (void)hadc;
}

// ── FFT处理：去直流 → 加窗 → FFT → 返回基频相位和频率 ──
static float fftGetPhase(const uint16_t *buf, float *outFreq)
{
    // 1. 去直流
    float dc = 0.0f;
    for (int i = 0; i < PLL_FFT_NUM; i++) dc += (float)buf[i];
    dc /= PLL_FFT_NUM;

    // 2. 构建复数输入
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        s_fftBuf[2 * i]     = (float)buf[i] - dc;
        s_fftBuf[2 * i + 1] = 0.0f;
    }

    // 3. FFT (原地)
    arm_cfft_f32(&s_cfftInst, s_fftBuf, 0, 1);

    // 4. 幅度谱
    arm_cmplx_mag_f32(s_fftBuf, s_fftMag, PLL_FFT_NUM);

    // 5. 找基频峰值
    uint16_t peakIdx = findPeak(s_fftMag, 2, PLL_FFT_NUM / 2);

    if (outFreq) {
        *outFreq = (float)peakIdx * PLL_SAMPLE_RATE / PLL_FFT_NUM;
    }

    // 6. 相位: atan2(imag, real)
    return atan2f(s_fftBuf[2 * peakIdx + 1], s_fftBuf[2 * peakIdx]);
}

// ── 初始化 ────────────────────────────────────────────
void DPLL_Init(void)
{
    arm_cfft_init_f32(&s_cfftInst, PLL_FFT_NUM);

    // PID参数：Kp=3 Ki=0.02 Kd=0.1 限制放宽到±500Hz
    pid_init(&s_phasePid, 3.0f, 0.02f, 0.1f, 500.0f, 500.0f, 0.0f);

    // TIM3 = 256kHz (200MHz / 781 ≈ 256.1kHz)
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 781 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 首次启动
    s_dataReady = 0;
    s_firstRun = 1;
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
}

// ── 主循环处理 ────────────────────────────────────────
void DPLL_Loop(void)
{
    if (!s_dataReady) return;
    s_dataReady = 0;

    // 此时TIM3已停，缓冲区已冻结（回调里关掉了）
    // 停掉ADC DMA（释放DMA通道）
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    // ── FFT测输入信号相位和频率 ──
    float freqIn, phaseIn = fftGetPhase(g_adcIn, &freqIn);

    // ── FFT测反馈信号相位 ──
    float phaseOut = fftGetPhase(g_adcOut, NULL);

    // ── 相位误差 (归一化到[-π, π]) ──
    float phaseErr = phaseIn - phaseOut;
    while (phaseErr > M_PI)  phaseErr -= 2.0f * M_PI;
    while (phaseErr < -M_PI) phaseErr += 2.0f * M_PI;

    g_phaseView = phaseErr * 57.29578f;

    // ── 频率控制 ──
    if (s_firstRun) {
        // 首次：用FFT测得的频率作为初值，快速逼近
        g_ddsFreq = freqIn;
        pid_reset(&s_phasePid);
        s_firstRun = 0;
    } else {
        // PID对相位误差做微调
        float deltaFreq = pid_calc(&s_phasePid, 0.0f, phaseErr);
        g_ddsFreq -= deltaFreq;
    }

    if (g_ddsFreq > 150000.0f) g_ddsFreq = 150000.0f;
    if (g_ddsFreq < 50000.0f)  g_ddsFreq = 50000.0f;

    // ── 更新AD9833 (主循环中，可以安全调用阻塞SPI) ──
    AD9833_SetFrequency(&hds, g_ddsFreq);

    // ── 调试：ADC原始值范围 ──
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        if (g_adcIn[i]  > g_debugVinMax)  g_debugVinMax  = g_adcIn[i];
        if (g_adcIn[i]  < g_debugVinMin)  g_debugVinMin  = g_adcIn[i];
        if (g_adcOut[i] > g_debugVoutMax) g_debugVoutMax = g_adcOut[i];
        if (g_adcOut[i] < g_debugVoutMin) g_debugVoutMin = g_adcOut[i];
    }

    s_loopCnt++;

    // ── 重启采样（必须等DDS更新完再启动下一轮） ──
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
}
