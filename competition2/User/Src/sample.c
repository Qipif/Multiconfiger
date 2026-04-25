/**
 * sample.c — 数字锁相环(DPLL)
 * 方法：同步采样 + FFT测相 + PID频率控制
 * 架构：DMA ISR只置标志 + 清Cache，主循环做FFT/PID/DDS更新
 *
 * 硬件连接：
 *   ADC2 (PA6) = 外部输入信号 100kHz
 *   ADC1 (PB1) = AD9833输出反馈
 *   两路ADC由TIM3同步触发，通过DMA循环写入缓冲区
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
static float    s_fftBuf[2 * PLL_FFT_NUM];   // FFT输入/输出 (实虚交替)
static float    s_fftMag[PLL_FFT_NUM];       // 幅度谱
static arm_cfft_instance_f32 s_cfftInst;     // FFT实例
static pid_struct_t s_phasePid;              // PID控制器

static volatile uint8_t s_dataReady = 0;    // 0=无新数据 1=半满 2=全满
static uint32_t s_loopCnt = 0;              // PID迭代计数

// 外部引用
extern AD9833_Handler  hds;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

// ── 工具函数：寻找幅度谱峰值 ─────────────────────────
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

// ── DMA半传输完成回调 ─────────────────────────────────
//    只做两件事: ①清DCache ②置标志
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    // H7关键：Invalidate DCache，读到DMA刚写入的数据
    uint32_t addr = (uint32_t)g_adcIn & ~(uint32_t)0x1F;
    uint32_t size = PLL_FFT_NUM * sizeof(uint16_t);
    size = (size + 31) & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

    addr = (uint32_t)g_adcOut & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

    s_dataReady = 1;
}

// ── DMA传输完成回调 ───────────────────────────────────
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    uint32_t addr = (uint32_t)g_adcIn & ~(uint32_t)0x1F;
    uint32_t size = PLL_FFT_NUM * sizeof(uint16_t);
    size = (size + 31) & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

    addr = (uint32_t)g_adcOut & ~(uint32_t)0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

    s_dataReady = 2;
}

// ── 初始化 ────────────────────────────────────────────
void DPLL_Init(void)
{
    // 初始化CMSIS-DSP FFT实例
    arm_cfft_init_f32(&s_cfftInst, PLL_FFT_NUM);

    // 初始化PID：Kp=3.0 Ki=0.02 Kd=0.1 (参考已验证参数)
    pid_init(&s_phasePid, 3.0f, 0.02f, 0.1f, 12.0f, 16.0f, 0.0f);

    // TIM3采样率设为256kHz
    // TIM3在APB1，H7上APB1 prescaler≠1时 timer时钟=2×APB1=200MHz
    // 200MHz / 781 ≈ 256.1kHz ≈ 256kHz
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 781 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start(&htim3);

    // 启动双通道ADC DMA (循环模式)
    s_dataReady = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
}

// ── FFT处理单个通道，返回基频相位 ──────────────────────
static float fftGetPhase(const uint16_t *adcBuf, float *outFreq)
{
    // 1. 去直流
    float dc = 0.0f;
    for (int i = 0; i < PLL_FFT_NUM; i++) dc += (float)adcBuf[i];
    dc /= PLL_FFT_NUM;

    // 2. 构建复数输入 (实部=信号, 虚部=0)
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        s_fftBuf[2 * i]     = (float)adcBuf[i] - dc;
        s_fftBuf[2 * i + 1] = 0.0f;
    }

    // 3. 执行FFT (原地变换)
    arm_cfft_f32(&s_cfftInst, s_fftBuf, 0, 1);

    // 4. 计算幅度谱
    arm_cmplx_mag_f32(s_fftBuf, s_fftMag, PLL_FFT_NUM);

    // 5. 找基频峰值 (跳过DC bin 0, 以及附近的低频噪声)
    uint16_t peakIdx = findPeak(s_fftMag, 2, PLL_FFT_NUM / 2);

    // 6. 提取频率和相位
    if (outFreq) {
        *outFreq = (float)peakIdx * PLL_SAMPLE_RATE / PLL_FFT_NUM;
    }

    // atan2(imag, real) — CMSIS-DSP FFT输出格式: [real0, imag0, real1, imag1, ...]
    return atan2f(s_fftBuf[2 * peakIdx + 1], s_fftBuf[2 * peakIdx]);
}

// ── 主循环处理（在主函数while(1)中调用）────────────────
void DPLL_Loop(void)
{
    if (!s_dataReady) return;

    uint8_t flag = s_dataReady;
    s_dataReady = 0;

    // ── 从DMA缓冲区复制到栈上（防止处理中被DMA覆盖）───
    uint16_t procIn[PLL_FFT_NUM];
    uint16_t procOut[PLL_FFT_NUM];
    memcpy(procIn,  g_adcIn,  sizeof(procIn));
    memcpy(procOut, g_adcOut, sizeof(procOut));

    // ── FFT测相：输入信号 ──
    float freqIn;
    float phaseIn = fftGetPhase(procIn, &freqIn);

    // ── FFT测相：DDS输出反馈 ──
    float freqOut_unused;
    float phaseOut = fftGetPhase(procOut, &freqOut_unused);

    // ── 计算相位误差 ──
    float phaseErr = phaseIn - phaseOut;
    while (phaseErr > M_PI)  phaseErr -= 2.0f * M_PI;
    while (phaseErr < -M_PI) phaseErr += 2.0f * M_PI;

    g_phaseView = phaseErr * 57.29578f;  // rad → deg

    // ── PID控制：相位误差 → 频率修正量 ──
    //    参考方案：g_totalFreq = g_baseFreq - pid_calc(pid, 0, phaseErr)
    float deltaFreq = pid_calc(&s_phasePid, 0.0f, phaseErr);
    g_ddsFreq -= deltaFreq;

    // ── 频率限幅 ──
    if (g_ddsFreq > 150000.0f) g_ddsFreq = 150000.0f;
    if (g_ddsFreq < 50000.0f)  g_ddsFreq = 50000.0f;

    // ── 更新AD9833 ──
    AD9833_SetFrequency(&hds, g_ddsFreq);

    // ── 调试：记录ADC原始值范围 ──
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        if (procIn[i]  > g_debugVinMax)  g_debugVinMax  = procIn[i];
        if (procIn[i]  < g_debugVinMin)  g_debugVinMin  = procIn[i];
        if (procOut[i] > g_debugVoutMax) g_debugVoutMax = procOut[i];
        if (procOut[i] < g_debugVoutMin) g_debugVoutMin = procOut[i];
    }

    s_loopCnt++;
}
