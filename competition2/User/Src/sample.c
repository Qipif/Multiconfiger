/**
 * sample.c - 时域锁相环
 * 参考 01_signal_separator 方案：
 *   - 双ADC同步采样（TIM3触发，DMA_NORMAL）
 *   - 时域过零检测测频 + 测相
 *   - PID调频，驱动AD9833输出同频信号
 */

#include "sample.h"
#include "ad9833.h"
#include "oled.h"
#include <stdio.h>
#include <math.h>

// ── 全局变量 ──────────────────────────────────────────────────
uint8_t  g_syncFlag  = 0x00;   // bit0=ADC1完成, bit1=ADC2完成
PhaseState g_phaseState = PHASE_IDLE;
float    g_measuredFreq  = 0.0f;
float    g_measuredPhase = 0.0f;   // 度，输入-输出
float    g_ddsFreq = 100000.0f;   // DDS输出频率（供mainoop.c显示）
extern   AD9833_Handler hds;      // 在mainoop.c定义
uint16_t g_adcIn[SAMPLE_NUM];      // ADC2: 输入信号 PA6
uint16_t g_adcOut[SAMPLE_NUM];     // ADC1: DDS反馈 PB1

// ── 静态变量 ─────────────────────────────────────────────────
static float s_pidIntegral = 0.0f;
static uint8_t s_lockCnt = 0;

// ── DCache Invalidate（H743 DCache安全） ─────────────────────
static void InvalidateADC(void)
{
    // SRAM1，Cache行对齐
    uint32_t addrIn  = (uint32_t)g_adcIn;
    uint32_t addrOut = (uint32_t)g_adcOut;
    uint32_t sizeIn  = SAMPLE_NUM * sizeof(uint16_t);
    uint32_t sizeOut = SAMPLE_NUM * sizeof(uint16_t);

    // 32字节对齐
    addrIn  &= ~0x1F;
    addrOut &= ~0x1F;
    sizeIn   = (sizeIn  + 31) & ~0x1F;
    sizeOut  = (sizeOut + 31) & ~0x1F;

    SCB_InvalidateDCache_by_Addr((uint32_t*)addrIn,  sizeIn);
    SCB_InvalidateDCache_by_Addr((uint32_t*)addrOut, sizeOut);
}

// ── ADC DMA 完成回调 ──────────────────────────────────────────
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2) {
        g_syncFlag |= 0x01;   // ADC2完成
    }
    else if (hadc->Instance == ADC1) {
        g_syncFlag |= 0x02;   // ADC1完成
    }
}

// ── 启动一次双ADC同步采样 ─────────────────────────────────────
void phaseLockStart(void)
{
    g_syncFlag = 0x00;

    InvalidateADC();

    // DMA NORMAL：每次需重新启动
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_adcIn,  SAMPLE_NUM);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adcOut, SAMPLE_NUM);
    // TIM3已在main_init启动，TRGO=Update周期性触发ADC
}

// ── 停止采样 ──────────────────────────────────────────────────
void phaseLockStop(void)
{
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc1);
    g_phaseState = PHASE_IDLE;
}

// ── 时域测频（过零检测） ──────────────────────────────────────
static float MeasureFreq(void)
{
    // 找第一个上升过零点
    uint16_t mid = 2048;  // 12bit ADC中点
    int32_t firstZ = -1, nextZ = -1;

    for (int i = 1; i < SAMPLE_NUM && (nextZ == -1); i++) {
        if (g_adcIn[i-1] < mid && g_adcIn[i] >= mid) {
            // 线性插值求精确过零位置
            float frac = (float)(mid - g_adcIn[i-1]) /
                         (float)(g_adcIn[i] - g_adcIn[i-1]);
            int32_t z = i - 1 + (int32_t)frac;

            if (firstZ == -1) {
                firstZ = z;
            } else {
                nextZ = z;
            }
        }
    }

    if (firstZ != -1 && nextZ != -1) {
        float period_samples = (float)(nextZ - firstZ);
        float fs = TIM3_TRIGGER_FREQ;  // 采样率
        return fs / period_samples;
    }
    return g_measuredFreq;  // 失败则返回上次值
}

// ── 时域测相（过零相位差） ─────────────────────────────────────
static float MeasurePhase(void)
{
    uint16_t mid = 2048;
    int32_t zIn = -1, zOut = -1;

    // 输入信号过零
    for (int i = 1; i < SAMPLE_NUM && zIn == -1; i++) {
        if (g_adcIn[i-1] < mid && g_adcIn[i] >= mid) {
            float frac = (float)(mid - g_adcIn[i-1]) /
                         (float)(g_adcIn[i] - g_adcIn[i-1]);
            zIn = i - 1 + (int32_t)frac;
            break;
        }
    }

    // 输出信号过零
    for (int i = 1; i < SAMPLE_NUM && zOut == -1; i++) {
        if (g_adcOut[i-1] < mid && g_adcOut[i] >= mid) {
            float frac = (float)(mid - g_adcOut[i-1]) /
                         (float)(g_adcOut[i] - g_adcOut[i-1]);
            zOut = i - 1 + (int32_t)frac;
            break;
        }
    }

    if (zIn != -1 && zOut != -1) {
        float diff_samples = (float)(zOut - zIn);
        if (diff_samples < 0) diff_samples += SAMPLE_NUM;
        float phase_deg = diff_samples / SAMPLE_NUM * 360.0f;
        return phase_deg;
    }
    return g_measuredPhase;
}

// ── PID 调频 ──────────────────────────────────────────────────
static void PidAdjust(float freqError)
{
    // 比例项
    float Pout = PID_KP * freqError;

    // 积分项（限幅）
    s_pidIntegral += freqError;
    if (s_pidIntegral > 5000.0f)  s_pidIntegral = 5000.0f;
    if (s_pidIntegral < -5000.0f) s_pidIntegral = -5000.0f;
    float Iout = PID_KI * s_pidIntegral;

    // 新的DDS频率
    g_ddsFreq += Pout + Iout;

    // 限幅：只允许在目标频率附近微调
    if (g_ddsFreq > TARGET_FREQ + 5000.0f) g_ddsFreq = TARGET_FREQ + 5000.0f;
    if (g_ddsFreq < TARGET_FREQ - 5000.0f) g_ddsFreq = TARGET_FREQ - 5000.0f;

    // 设置AD9833频率
    AD9833_SetFrequency(&hds, (uint32_t)g_ddsFreq);
}

// ── 主状态机循环（在main_loop里周期性调用） ────────────────────
static uint32_t s_tick = 0;  // 超时检测

void sampleLoop(void)
{
    switch (g_phaseState) {

        case PHASE_IDLE:
            // 上电后自动开始
            g_phaseState = PHASE_SAMPLING;
            s_tick = HAL_GetTick();
            phaseLockStart();
            break;

        case PHASE_SAMPLING:
            // 等待两个ADC都完成
            if (g_syncFlag == 0x03) {
                g_phaseState = PHASE_PROC;
            }
            // 超时保护：50ms
            if (HAL_GetTick() - s_tick > 50) {
                phaseLockStart();  // 重新启动
                s_tick = HAL_GetTick();
            }
            break;

        case PHASE_PROC:
            InvalidateADC();

            // 时域测频
            g_measuredFreq = MeasureFreq();

            // 时域测相
            g_measuredPhase = MeasurePhase();

            // PID调频：让DDS频率跟踪输入频率
            {
                float freqError = g_measuredFreq - g_ddsFreq;
                PidAdjust(freqError);
            }

            // 判断锁定：频率误差<10Hz认为锁定
            if (fabsf(g_measuredFreq - g_ddsFreq) < 10.0f) {
                s_lockCnt++;
            } else {
                s_lockCnt = 0;
            }

            // OLED显示（由mainoop.c统一刷新，这里只更新数据）
            // 重新启动采样
            g_phaseState = PHASE_SAMPLING;
            s_tick = HAL_GetTick();
            phaseLockStart();
            break;

        default:
            g_phaseState = PHASE_IDLE;
            break;
    }
}
