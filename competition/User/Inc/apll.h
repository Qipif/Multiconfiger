#ifndef __APLL_H
#define __APLL_H

#include "main.h"
#include <stdint.h>

// APLL: 数字延迟线锁相（方波输出版）
// ADC采输入 → ring缓冲+延迟插值 → 输出方波(正半周高/负半周低)
// 同时钟 → 延迟=绝对时间 → 相位天然锁死
// 外部RC/LC低通滤波器把方波变回正弦波

#define APLL_RING_SIZE  2048   // 环形缓冲大小
#define APLL_RING_MASK  (APLL_RING_SIZE - 1)
#define APLL_DAC_BUF    256    // DAC输出缓冲 = ADC_BUF_LEN

typedef struct {
    float fs;                // 采样率

    // 相位控制
    float target_phase_deg;  // 目标相位偏移(度)
    float amp_scale;         // 幅度缩放 0~1

    // 延迟（核心）
    float delay_f;           // 当前延迟（浮点，样本数）
    float delay_target;      // 目标延迟
    float delay_alpha;       // 平滑系数
    float inherent_offset;   // 系统固有延迟偏移（校准值）

    // 零交叉测频
    float spp;               // 每周期采样点数（低通后）
    int16_t last_sample;     // 上次采样值（减中点）
    uint16_t last_zc_widx;   // 上次零交叉时widx
    uint8_t  zc_found;       // 是否已找到零交叉

    // 环形缓冲
    uint16_t ring[APLL_RING_SIZE];
    uint16_t widx;           // 写入位置

    // DAC输出缓冲
    uint16_t dac_buf[APLL_DAC_BUF];
} APLL_Handle;

// 初始化（fs=采样率）
void APLL_Init(APLL_Handle *h, float fs);

// 设置目标相位(度)和幅度(0~1)
void APLL_SetPhase(APLL_Handle *h, float deg, float amp);

// 单样本处理：写ring + 零交叉测频 + 延迟插值 → 方波输出
uint16_t APLL_Step(APLL_Handle *h, uint16_t adc);

// 处理半帧ADC数据，填DAC对应半边
void APLL_Process(APLL_Handle *h, uint16_t *adc_data,
                  uint16_t dac_offset, uint16_t len);

// 获取测到的输入频率
float APLL_GetFreq(APLL_Handle *h);

// 手动校准：在当前delay_f位置设为0°参考
void APLL_Calibrate(APLL_Handle *h);

#endif
