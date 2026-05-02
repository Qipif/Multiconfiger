/**
 * sample.c — 流式IQ Costas环 v10
 * FLL(零交叉测频) + PLL(Costas相位锁定) 双环结构
 *
 * FLL: 统计每个DMA缓冲区的上升过零数→测频→拉g_ddsFreq向实测值
 * PLL: Q*sign(I)鉴相→二阶环路滤波→精细相位调整
 *
 * 采样率256kHz, 100kHz信号每周期~2.56点
 * 256点缓冲区=1ms, 含~100个上升过零, 测频精度约±500Hz/帧
 */

#include "sample.h"
#include "ad9833.h"
#include "main.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926f
#endif

// ── 硬件参数 ──────────────────────────────────────
#define COS_TABLE_SIZE    1024
#define PHASE_SHIFT       22        // 32-bit相位取top 10bit查表
#define ADC_MID           2048.0f
#define ADC_SCALE         (1.0f / 2048.0f)

// ── IQ低通 ────────────────────────────────────────
#define IQ_ALPHA          0.05f     // IQ低通系数

// ── FLL参数 ───────────────────────────────────────
#define FLL_SMOOTH        0.15f     // 测频平滑系数（越大跟踪越快，但越抖）
#define FLL_PULL          0.2f      // FLL拉力（拉向测频值的力度，0.2=20%/帧）
#define FLL_MIN_RISE      20        // 最少上升过零数（低于此说明信号弱，不更新FLL）

// ── PLL参数 ───────────────────────────────────────
// FLL已把频率拉到附近，PLL只做精细相位调整
#define LOOP_KP          500.0f    // 比例增益（Q*sign(I)→Hz）
#define LOOP_KI          50.0f     // 积分增益
#define LOOP_I_MAX       2000.0f   // 积分限幅(Hz)，抗windup

// 频率限制（给FLL+PLL足够的调整空间）
#define FREQ_MIN           80000.0f
#define FREQ_MAX           120000.0f

// 相位死区
#define PHASE_DEADZONE     0.02f

// 锁定判定
#define LOCK_PHASE_THR     0.05f    // Q*sign(I)阈值
#define LOCK_FREQ_THR      10.0f    // Hz

// ── 全局变量 ──────────────────────────────────────
uint16_t g_adcIn[PLL_FFT_NUM];
uint16_t g_adcOut[PLL_FFT_NUM];
float    g_ddsFreq    = 100000.0f;
float    g_phaseView  = 0.0f;
float    g_freqMeas   = 100000.0f;  // 零交叉测频结果（平滑后）
float    g_debugVinMax  = 0.0f;
float    g_debugVinMin  = 4096.0f;
float    g_debugVoutMax = 0.0f;
float    g_debugVoutMin = 4096.0f;
uint32_t g_pllLoopCnt = 0;
uint32_t g_cbCnt      = 0;
uint8_t  g_isLocked   = 0;

// ── 内部状态 ──────────────────────────────────────
static float    s_cosTable[COS_TABLE_SIZE];
static uint32_t s_phaseAcc   = 0;
static float    s_I_filt     = 0.0f;
static float    s_Q_filt     = 0.0f;
static float    s_loopI      = 0.0f;   // PLL积分项(Hz)
static uint8_t  s_firstRun   = 1;

// FLL状态
static float    s_lastSig    = 0.0f;   // 上一个采样值（过零检测用）
static int      s_riseCount  = 0;      // 上升过零计数（全缓冲区累积）
static float    s_freqSmooth = 100000.0f;  // 测频平滑值

// FFT（仅首轮用）
static float           s_fftBuf[2 * PLL_FFT_NUM];
static float           s_fftMag[PLL_FFT_NUM];
static arm_cfft_instance_f32 s_fftInst;

static volatile uint8_t s_updateDds = 0;

extern AD9833_Handler  hds;
extern TIM_HandleTypeDef  htim3;
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;

// ── 逐点处理：IQ混频 + 过零检测（ISR里调用）──────────
static inline void iq_mix_point(float sig)
{
    // 上升过零检测（供FLL测频用）
    if (s_lastSig < 0.0f && sig >= 0.0f) {
        s_riseCount++;
    }
    s_lastSig = sig;

    // IQ混频
    uint32_t idx = s_phaseAcc >> PHASE_SHIFT;
    float I = sig * s_cosTable[idx];
    float Q = sig * s_cosTable[(idx + 768) & 0x3FF];  // sin = cos(idx+768)
    s_I_filt += IQ_ALPHA * (I - s_I_filt);
    s_Q_filt += IQ_ALPHA * (Q - s_Q_filt);

    // 推进NCO相位
    uint32_t step = (uint32_t)((double)g_ddsFreq / PLL_SAMPLE_RATE * 4294967296.0);
    s_phaseAcc += step;
}

static void process_chunk(const uint16_t *buf, int len)
{
    for (int i = 0; i < len; i++) {
        float sig = ((float)buf[i] - ADC_MID) * ADC_SCALE;
        iq_mix_point(sig);
    }
}

// ── DMA回调 ────────────────────────────────────────
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)((uint32_t)g_adcIn & ~(uint32_t)0x1F),
        ((PLL_FFT_NUM / 2 * sizeof(uint16_t)) + 31) & ~(uint32_t)0x1F);
    process_chunk(g_adcIn, PLL_FFT_NUM / 2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)g_adcIn + PLL_FFT_NUM/2) & ~(uint32_t)0x1F),
        ((PLL_FFT_NUM / 2 * sizeof(uint16_t)) + 31) & ~(uint32_t)0x1F);
    process_chunk(g_adcIn + PLL_FFT_NUM / 2, PLL_FFT_NUM / 2);
    s_updateDds = 1;
    g_cbCnt++;
}

// ── FFT粗测频（仅首轮用一次）──────────────────────
static float fft_measure_freq(void)
{
    float dc = 0.0f;
    for (int i = 0; i < PLL_FFT_NUM; i++) dc += (float)g_adcIn[i];
    dc /= PLL_FFT_NUM;
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        s_fftBuf[2*i]     = (float)g_adcIn[i] - dc;
        s_fftBuf[2*i + 1] = 0.0f;
    }
    arm_cfft_f32(&s_fftInst, s_fftBuf, 0, 1);
    arm_cmplx_mag_f32(s_fftBuf, s_fftMag, PLL_FFT_NUM);

    uint16_t peak = 2; float maxVal = s_fftMag[2];
    for (uint16_t i = 3; i < PLL_FFT_NUM/2; i++)
        if (s_fftMag[i] > maxVal) { maxVal = s_fftMag[i]; peak = i; }
    return (float)peak * PLL_SAMPLE_RATE / PLL_FFT_NUM;
}

// ── 初始化 ────────────────────────────────────────
void DPLL_Init(void)
{
    // 余弦查表
    for (int i = 0; i < COS_TABLE_SIZE; i++)
        s_cosTable[i] = cosf(2.0f * M_PI * i / COS_TABLE_SIZE);

    // FFT初始化（首轮用）
    arm_cfft_init_f32(&s_fftInst, PLL_FFT_NUM);

    // TIM3 = 256kHz (200MHz / 781 ≈ 256kHz)
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 781 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 状态清零
    s_phaseAcc   = 0;
    s_I_filt     = 0.0f;
    s_Q_filt     = 0.0f;
    s_loopI      = 0.0f;
    s_lastSig    = 0.0f;
    s_riseCount  = 0;
    s_freqSmooth = 100000.0f;
    s_firstRun   = 1;
    g_pllLoopCnt = 0;
    g_cbCnt      = 0;
    g_isLocked   = 0;
    s_updateDds  = 0;

    // 启动ADC DMA + TIM3
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  PLL_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, PLL_FFT_NUM);
    HAL_TIM_Base_Start(&htim3);
}

// ── 主循环（FLL + PLL 双环）────────────────────────
void DPLL_Loop(void)
{
    if (!s_updateDds) return;
    s_updateDds = 0;

    // 首轮FFT粗测频
    if (s_firstRun) {
        float f = fft_measure_freq();
        if (f > 5000.0f) {
            g_ddsFreq = f;
            s_freqSmooth = f;
            s_loopI = 0.0f;
        }
        s_firstRun = 0;
    }

    // ── 1. FLL：零交叉测频 → 频率锚 ──
    //    统计整个缓冲区的上升过零数→算频率
    //    256点@256kHz = 1ms, 100kHz信号≈100个上升过零
    int riseCount = s_riseCount;
    s_riseCount = 0;  // 重置计数器

    if (riseCount >= FLL_MIN_RISE) {
        float duration = (float)PLL_FFT_NUM / PLL_SAMPLE_RATE;  // 1ms
        float freqRaw = (float)riseCount / duration;

        // 平滑测频值
        s_freqSmooth += FLL_SMOOTH * (freqRaw - s_freqSmooth);
    }
    g_freqMeas = s_freqSmooth;

    // FLL粗调：把频率拉向实测值
    //    freqMeas > g_ddsFreq → correction > 0 → 频率上升 → 正确
    float fllCorr = FLL_PULL * (s_freqSmooth - g_ddsFreq);
    g_ddsFreq += fllCorr;

    // ── 2. PLL：Costas环相位锁定 ──
    //    Q*sign(I)鉴相器：
    //    NCO偏慢 → phaseErr > 0 → 需提高频率
    //    NCO偏快 → phaseErr < 0 → 需降低频率
    float phaseErr = s_Q_filt * (s_I_filt >= 0.0f ? 1.0f : -1.0f);

    // 信号太弱时清零（防噪声驱动环路）
    float mag2 = s_I_filt * s_I_filt + s_Q_filt * s_Q_filt;
    if (mag2 < 0.001f) phaseErr = 0.0f;

    // 相位死区（锁定后防抖）
    if (fabsf(phaseErr) < PHASE_DEADZONE)
        phaseErr = 0.0f;

    // 相位显示（用atan2算真实角度，仅显示不参与控制）
    g_phaseView = atan2f(s_Q_filt, s_I_filt) * 57.29578f;

    // 二阶PLL环路滤波器（Type-II）
    s_loopI += LOOP_KI * phaseErr;

    // 积分限幅（抗windup）
    if (s_loopI >  LOOP_I_MAX) s_loopI =  LOOP_I_MAX;
    if (s_loopI < -LOOP_I_MAX) s_loopI = -LOOP_I_MAX;

    float pllCorr = LOOP_KP * phaseErr + s_loopI;

    // phaseErr > 0 → NCO偏慢 → 需提高频率 → 加号
    // ⚠️ 如果锁反了（频率往错误方向跑），把 += 改成 -=
    g_ddsFreq += pllCorr;

    // 限幅
    if (g_ddsFreq > FREQ_MAX) g_ddsFreq = FREQ_MAX;
    if (g_ddsFreq < FREQ_MIN) g_ddsFreq = FREQ_MIN;

    AD9833_SetFrequency(&hds, g_ddsFreq);

    // ── 3. 锁定判定 ──
    g_isLocked = (fabsf(phaseErr) < LOCK_PHASE_THR &&
                  fabsf(pllCorr + fllCorr) < LOCK_FREQ_THR) ? 1 : 0;

    // ── 4. 调试信息 ──
    for (int i = 0; i < PLL_FFT_NUM; i++) {
        if (g_adcIn[i]  > g_debugVinMax)  g_debugVinMax  = g_adcIn[i];
        if (g_adcIn[i]  < g_debugVinMin)  g_debugVinMin  = g_adcIn[i];
        if (g_adcOut[i] > g_debugVoutMax) g_debugVoutMax = g_adcOut[i];
        if (g_adcOut[i] < g_debugVoutMin) g_debugVoutMin = g_adcOut[i];
    }
    g_pllLoopCnt++;
}
