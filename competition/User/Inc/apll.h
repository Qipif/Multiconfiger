#ifndef __APLL_H
#define __APLL_H

#include "main.h"
#include <stdint.h>

// APLL v11: 极限抗抖版
// 核心改进（vs v10）：
//   1. 亚采样零交叉插值 → spp精度从1点→0.01点
//   2. NCO相位累加器输出 → 不依赖瞬时符号，消除过零抖
//   3. 锁定冻结机制 → 锁住后频率不再追噪声
//   4. ★ ZC相位纠偏 → 死区(±7°) + 极低增益(0.02/0.005) → 不震荡
//   5. IIR数字低通 → 消方波振铃/假ZC

#define APLL_RING_SIZE  2048   // 环形缓冲大小
#define APLL_RING_MASK  (APLL_RING_SIZE - 1)
#define APLL_DAC_BUF    256    // DAC输出缓冲 = ADC_BUF_LEN

typedef struct {
    float fs;                // 采样率

    // 相位控制
    float target_phase_deg;  // 目标相位偏移(度)
    float amp_scale;         // 幅度缩放 0~2

    // ── 亚采样零交叉 ──
    float spp;               // 每周期采样点数（低通后）
    float last_sample;       // 上次采样值（减中点，浮点，IIR低通后）
    float last_zc_pos;       // 上次零交叉的浮点位置（亚采样精度）
    float lp_state;          // IIR低通状态（消振铃/假ZC）
    uint8_t  zc_found;       // 是否已找到零交叉

    // ── NCO相位累加 ──
    float freq_filt;         // 滤波后的频率（Hz）
    float phase;             // NCO相位 [0, 1)

    // ── 锁定机制 ──
    uint8_t  locked;         // 是否已锁定
    uint8_t  lock_cnt;       // 连续锁定计数

    // ── 施密特触发 ──
    int8_t   last_out;       // 上次输出状态 (+1/-1)

    // ── 校准 ──
    float inherent_offset;   // 系统固有延迟偏移（校准值）
    float delay_f;           // 当前延迟（校准用）

    // 环形缓冲（保留，用于校准时参考）
    uint16_t ring[APLL_RING_SIZE];
    uint16_t widx;           // 写入位置

    // DAC输出缓冲
    uint16_t dac_buf[APLL_DAC_BUF];
} APLL_Handle;

// 初始化（fs=采样率）
void APLL_Init(APLL_Handle *h, float fs);

// 设置目标相位(度)和幅度(0~2)
void APLL_SetPhase(APLL_Handle *h, float deg, float amp);

// 单样本处理：ZC测频 + NCO累加 → 方波输出
uint16_t APLL_Step(APLL_Handle *h, uint16_t adc);

// 处理半帧ADC数据，填DAC对应半边
void APLL_Process(APLL_Handle *h, uint16_t *adc_data,
                  uint16_t dac_offset, uint16_t len);

// 获取测到的输入频率
float APLL_GetFreq(APLL_Handle *h);

// 手动校准：在当前位置设为0°参考
void APLL_Calibrate(APLL_Handle *h);

#endif
