#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "tim.h"

// 编码器按键事件
typedef enum {
    ENC_EVT_NONE = 0,
    ENC_EVT_CLICK,       // 短按
    ENC_EVT_LONG_PRESS,  // 长按(>1s)
    ENC_EVT_DOUBLE_CLICK // 双击
} ENC_Event_t;

// 编码器控制目标
typedef enum {
    ENC_CTRL_FREQ = 0,   // 调频率
    ENC_CTRL_AMP,        // 调幅度
    ENC_CTRL_PHASE,      // 调相位
    ENC_CTRL_WAVE,       // 切波形
    ENC_CTRL_PRESET,     // 选预设
    ENC_CTRL_FFT_MODE    // 切FFT模式
} ENC_CtrlTarget_t;

// 连续档位参数
typedef struct {
    float step_slow;     // 慢速步进
    float step_mid;      // 中速步进
    float step_fast;     // 快速步进
    float val_min;       // 最小值
    float val_max;       // 最大值
} ENC_ParamRange;

// 编码器状态
typedef struct {
    TIM_HandleTypeDef *htim;
    int16_t  last_cnt;
    int32_t  total;
    int32_t  remain;

    // 按键检测
    uint16_t gpio_pin;
    GPIO_TypeDef *gpio_port;
    uint32_t press_start;
    uint8_t  press_active;
    uint8_t  click_count;

    // 当前控制目标
    ENC_CtrlTarget_t target;

    // 各参数范围
    ENC_ParamRange freq_range;
    ENC_ParamRange amp_range;
    ENC_ParamRange phase_range;

    // 当前值
    float cur_freq;
    float cur_amp;
    float cur_phase;
    int cur_wave;
    int cur_fft_mode;
    uint8_t rotated;     // 1=本次Update有旋转
} ENC_Handle;

// 初始化
void ENC_Init(ENC_Handle *h, TIM_HandleTypeDef *htim,
                GPIO_TypeDef *port, uint16_t pin);

// 更新（主循环调用，检测旋转+按键）
ENC_Event_t ENC_Update(ENC_Handle *h);

// 应用编码器值到DDS（12项目不用）
// void ENC_ApplyToDDS(ENC_Handle *h, DDS_Handle *dds);

// 设置控制目标
void ENC_SetTarget(ENC_Handle *h, ENC_CtrlTarget_t target);
ENC_CtrlTarget_t ENC_GetTarget(ENC_Handle *h);

// 切换控制目标（循环：频率→幅度→相位→波形→FFT模式→频率）
void ENC_NextTarget(ENC_Handle *h);

// 获取旋转速度（档位：0=慢 1=中 2=快）
uint8_t ENC_GetGear(ENC_Handle *h);

// 获取当前值字符串
const char* ENC_TargetStr(ENC_CtrlTarget_t t);

#endif
