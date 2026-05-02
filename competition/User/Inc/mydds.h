#ifndef __MYDDS_H__
#define __MYDDS_H__

#include "main.h"
#include "tim.h"
#include "dac.h"

// 波形类型
typedef enum {
    DDS_SINE = 0,
    DDS_TRIANGLE,
    DDS_SQUARE
} DDS_Wave_t;

// 算法类型
typedef enum {
    DDS_ALGO_PRECISE = 0,  // 高精度相位累加（适合100kHz）
    DDS_ALGO_FAST          // 快速查表256点（适合10kHz以下）
} DDS_Algo_t;

// 独立调节参数
typedef enum {
    DDS_PARAM_FREQ,
    DDS_PARAM_AMP,
    DDS_PARAM_PHASE
} DDS_Param_t;

// DDS配置（用于预设存储）
typedef struct {
    DDS_Algo_t  algo;
    DDS_Wave_t  wave;
    uint16_t      table_size;       // 查表点数(128/256/512)
    uint32_t      target_rate;      // DAC更新率(5~20MHz)
    float         auto_switch_freq; // 自动切换频率阈值(0=关闭)
    uint8_t       enable_interp;    // 线性插值开关
} DDS_Config;

// DDS句柄
typedef struct {
    TIM_HandleTypeDef *htim;
    DAC_HandleTypeDef *hdac;
    uint32_t          dac_channel;

    DDS_Config  cfg;
    DDS_Wave_t  wave;
    float         freq;
    float         amp_vpp;
    float         phase_deg;
    uint32_t      phase_acc;

    uint16_t *active_buf;
    uint16_t  buf0[512];
    uint16_t  buf1[512];
    uint32_t  buf_len;
    volatile uint8_t buf_idx;  // 0=buf0, 1=buf1

    float     process_ms;  // 上次处理耗时
} DDS_Handle;

// 初始化
void DDS_Init(DDS_Handle *h, TIM_HandleTypeDef *htim,
                DAC_HandleTypeDef *hdac, uint32_t ch);
void DDS_InitWithConfig(DDS_Handle *h, const DDS_Config *cfg,
                           TIM_HandleTypeDef *htim,
                           DAC_HandleTypeDef *hdac, uint32_t ch);

// 设置波形（全参数）
void DDS_SetWave(DDS_Handle *h, DDS_Wave_t wave,
                   float freq, float amp_vpp, float phase_deg);

// 独立调参（快速路径）
void DDS_SetFreq(DDS_Handle *h, float freq);
void DDS_SetAmp(DDS_Handle *h, float amp_vpp);
void DDS_SetPhase(DDS_Handle *h, float phase_deg);
void DDS_SetParam(DDS_Handle *h, DDS_Param_t param, float val);

// 切换算法
void DDS_SetAlgo(DDS_Handle *h, DDS_Algo_t algo);

// 应用配置
void DDS_ApplyConfig(DDS_Handle *h, const DDS_Config *cfg);
void DDS_GetConfig(DDS_Handle *h, DDS_Config *cfg);

// 启停
void DDS_Start(DDS_Handle *h);
void DDS_Stop(DDS_Handle *h);

// 工具
const char* DDS_AlgoStr(DDS_Algo_t a);
const char* DDS_WaveStr(DDS_Wave_t w);

// ========================================================================
// 实时DDS（RT）- DMA永不停，相位下一采样点即生效
// ========================================================================
#define DDS_RT_BUF_LEN 128  // 双缓冲，每半帧实时计算

typedef struct {
    TIM_HandleTypeDef *htim;
    DAC_HandleTypeDef *hdac;

    // ch1
    DDS_Wave_t  wave1;
    float        freq1;
    float        amp1_vpp;
    uint16_t     amp1_scale_q12;  // Q12定点幅度（ISR直接用）
    float        phase1_deg;

    // ch2
    DDS_Wave_t  wave2;
    float        freq2;
    float        amp2_vpp;
    uint16_t     amp2_scale_q12;  // Q12定点幅度
    float        phase2_deg;

    // 实时状态
    uint32_t     phase_acc1;
    uint32_t     phase_acc2;
    uint32_t     phase_step1;
    uint32_t     phase_step2;
    uint32_t     phase_off2;   // ch2相位偏移Q32，修改即时生效

    uint16_t     buf1[DDS_RT_BUF_LEN];
    uint16_t     buf2[DDS_RT_BUF_LEN];
} DDS_RT_Handle;

// 初始化（启动TIM+DMA循环，填充初始buf）
void DDS_RT_Init(DDS_RT_Handle *h, TIM_HandleTypeDef *htim, DAC_HandleTypeDef *hdac,
                 float freq, float amp_vpp, float phase2_deg);

// 实时调参（原子操作，下一采样点生效）
void DDS_RT_SetFreq(DDS_RT_Handle *h, float freq);
void DDS_RT_SetAmp(DDS_RT_Handle *h, float amp_vpp);
void DDS_RT_SetPhase2(DDS_RT_Handle *h, float phase2_deg);
void DDS_RT_SetWave2(DDS_RT_Handle *h, DDS_Wave_t wave);

// 填上半帧（DMA半传输完成回调中调用）
void DDS_RT_FillHalf(DDS_RT_Handle *h);
// 填上下半帧（DMA传输完成回调中调用）
void DDS_RT_FillOther(DDS_RT_Handle *h);

#endif
