#include "mypll.h"
#include <math.h>

#ifndef M_PI_F
#define M_PI_F 3.1415926f
#endif

// AD9959 频率字转换常数（系统时钟500MHz）
#define FTW_CONSTANT 8.589934592f  // 2^32 / 500e6

static inline uint32_t freq_to_ftw(float freq_hz) {
    return (uint32_t)(freq_hz * FTW_CONSTANT);
}

static inline uint16_t rad_to_phase_word(float phase_rad) {
    while (phase_rad < 0) phase_rad += 2.0f * M_PI_F;
    while (phase_rad >= 2.0f * M_PI_F) phase_rad -= 2.0f * M_PI_F;
    return (uint16_t)(phase_rad * 16384.0f / (2.0f * M_PI_F)) & 0x3FFF;
}

/**
 * @brief 计算两个相位的最小弧度差，正确处理 ±π 边界跳变
 */
static float phase_difference(float phase1, float phase2) {
    float diff = phase1 - phase2;
    while (diff > M_PI_F)  diff -= 2.0f * M_PI_F;
    while (diff < -M_PI_F) diff += 2.0f * M_PI_F;
    return diff;
}

void PLL_Init(PLL_Handle *pll, float base_freq, float sample_rate, uint16_t fft_size,
              float freq_kp, float freq_ki, float freq_kd,
              float phase_kp, float phase_ki, float phase_kd) {
    if (!pll) return;

    pll->base_freq = base_freq;
    pll->sample_rate = sample_rate;
    pll->fft_size = fft_size;
    pll->dt = 1.0f / sample_rate;

    pll->freq_correction = 0.0f;
    pll->output_freq = base_freq;
    pll->freq_word = freq_to_ftw(base_freq);

    pll->phase_correction = 0.0f;
    pll->phase_word = 0;
    pll->accumulated_phase = 0.0f;

    pll->current_phase_err = 0.0f;
    pll->last_phase_err = 0.0f;
    pll->phase_rate = 0.0f;

    pid_init(&pll->freq_pid, freq_kp, freq_ki, freq_kd, 10.0f, 10.0f, 0.0f);
    pid_init(&pll->phase_pid, phase_kp, phase_ki, phase_kd, M_PI_F, M_PI_F, 0.0f);
}

void PLL_Update(PLL_Handle *pll, float ref_phase, float out_phase,
                uint32_t *new_freq_word, uint16_t *new_phase_word) {
    if (!pll) return;

    // 1. 计算当前相位误差
    pll->current_phase_err = phase_difference(ref_phase, out_phase);

    // 2. 计算相位变化率（这才是真正的频率误差指标）
    float delta_err = phase_difference(pll->current_phase_err, pll->last_phase_err);
    pll->phase_rate = delta_err / pll->dt;
    pll->last_phase_err = pll->current_phase_err;

    // 3. 频率误差 = 相位变化率 / (2π)
    float freq_error = pll->phase_rate / (2.0f * M_PI_F);

    // 4. 频率环PID（核心：只调节频率）
    pll->freq_correction = -pid_calc(&pll->freq_pid, 0.0f, freq_error);
    pll->output_freq = pll->base_freq + pll->freq_correction;

    if (pll->output_freq < 1.0f) pll->output_freq = 1.0f;
    if (pll->output_freq > 500000.0f) pll->output_freq = 500000.0f;

    pll->freq_word = freq_to_ftw(pll->output_freq);

    // 5. 相位字保持为0（不调节相位偏移）
    //    频率锁定后，相位差自然稳定
    pll->phase_word = 0;

    // 6. 输出
    if (new_freq_word) *new_freq_word = pll->freq_word;
    if (new_phase_word) *new_phase_word = pll->phase_word;
}

void PLL_Reset(PLL_Handle *pll) {
    if (!pll) return;
    pid_reset(&pll->freq_pid);
    pid_reset(&pll->phase_pid);
    pll->freq_correction = 0.0f;
    pll->output_freq = pll->base_freq;
    pll->freq_word = freq_to_ftw(pll->base_freq);
    pll->phase_correction = 0.0f;
    pll->phase_word = 0;
    pll->accumulated_phase = 0.0f;
    pll->current_phase_err = 0.0f;
    pll->last_phase_err = 0.0f;
    pll->phase_rate = 0.0f;
}
