#ifndef __MY_PLL_H__
#define __MY_PLL_H__

#include <stdint.h>
#include "pid.h"

typedef struct {
    // 配置参数
    float base_freq;            // 固定输出频率基准 (Hz)
    float sample_rate;          // PLL迭代速率 ≈ ADC完成频率 (Hz)
    uint16_t fft_size;

    // 频率环PID
    pid_struct_t freq_pid;
    float freq_correction;      // 频率修正量 (Hz)
    float output_freq;          // 当前输出频率 (Hz)
    uint32_t freq_word;         // AD9959频率字

    // 相位环PID
    pid_struct_t phase_pid;
    float phase_correction;     // 相位修正量 (rad)
    uint16_t phase_word;        // AD9959相位字 (0~16383)

    // 状态变量
    float current_phase_err;    // 当前相位误差 (rad)
    float last_phase_err;       // 上次相位误差 (rad)
    float phase_rate;           // 相位变化率 (rad/s)
    float dt;                   // 迭代时间间隔 (s)
    float accumulated_phase;    // 累积相位 (rad)
} PLL_Handle;

void PLL_Init(PLL_Handle *pll, float base_freq, float sample_rate, uint16_t fft_size,
              float freq_kp, float freq_ki, float freq_kd,
              float phase_kp, float phase_ki, float phase_kd);

void PLL_Update(PLL_Handle *pll, float ref_phase, float out_phase,
                uint32_t *new_freq_word, uint16_t *new_phase_word);

void PLL_Reset(PLL_Handle *pll);

#endif
