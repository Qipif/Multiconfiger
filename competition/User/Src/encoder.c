#include "encoder.h"
#include <string.h>
#include <math.h>

#define CLICK_TIMEOUT   300   // 双击间隔ms
#define LONG_PRESS_MS   1000  // 长按阈值

void ENC_Init(ENC_Handle *h, TIM_HandleTypeDef *htim,
                GPIO_TypeDef *port, uint16_t pin) {
    memset(h, 0, sizeof(ENC_Handle));
    h->htim = htim;
    h->gpio_port = port;
    h->gpio_pin = pin;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0);

    // 默认参数范围
    h->freq_range = (ENC_ParamRange){10.0f, 100.0f, 1000.0f, 10.0f, 200000.0f};
    h->amp_range  = (ENC_ParamRange){0.1f,  0.5f,  1.0f,   0.0f,  3.3f};
    h->phase_range= (ENC_ParamRange){10.0f, 30.0f, 90.0f,  0.0f,  360.0f};

    // 默认值
    h->cur_freq = 1000.0f;
    h->cur_amp = 2.0f;
    h->cur_phase = 0.0f;
    h->cur_wave = 0;  // SINE
    h->cur_fft_mode = 0;  // PRECISE
    h->target = ENC_CTRL_WAVE;
}

ENC_Event_t ENC_Update(ENC_Handle *h) {
    ENC_Event_t evt = ENC_EVT_NONE;

    // 旋转检测
    int16_t cur = (int16_t)__HAL_TIM_GET_COUNTER(h->htim);
    int16_t diff = cur - h->last_cnt;
    h->last_cnt = cur;
    h->remain += diff;
    int16_t delta = h->remain / 4;  // 4倍频
    h->remain %= 4;
    h->total += delta;

    if (delta != 0) {
        h->rotated = 1;

        // 数值型参数: FREQ/AMP/PHASE
        if (h->target == ENC_CTRL_FREQ || h->target == ENC_CTRL_AMP || h->target == ENC_CTRL_PHASE) {
            ENC_ParamRange *r = NULL;
            switch (h->target) {
            case ENC_CTRL_FREQ:  r = &h->freq_range; break;
            case ENC_CTRL_AMP:   r = &h->amp_range; break;
            case ENC_CTRL_PHASE: r = &h->phase_range; break;
            default: break;
            }
            if (r) {
                uint8_t gear = ENC_GetGear(h);
                float step = (gear == 0) ? r->step_slow : (gear == 1) ? r->step_mid : r->step_fast;
                step *= delta;
                switch (h->target) {
                case ENC_CTRL_FREQ:
                    h->cur_freq += step;
                    if (h->cur_freq < r->val_min) h->cur_freq = r->val_min;
                    if (h->cur_freq > r->val_max) h->cur_freq = r->val_max;
                    break;
                case ENC_CTRL_AMP:
                    h->cur_amp += step;
                    if (h->cur_amp < r->val_min) h->cur_amp = r->val_min;
                    if (h->cur_amp > r->val_max) h->cur_amp = r->val_max;
                    break;
                case ENC_CTRL_PHASE:
                    h->cur_phase += step;
                    if (h->cur_phase < r->val_min) h->cur_phase = r->val_min;
                    if (h->cur_phase > r->val_max) h->cur_phase = r->val_max;
                    break;
                default: break;
                }
            }
        }

        // 枚举型参数: WAVE/FFT_MODE
        if (h->target == ENC_CTRL_WAVE) {
            if (delta > 0)
                h->cur_wave = (h->cur_wave + 1) % 3;
            else
                h->cur_wave = (h->cur_wave + 2) % 3;
        }
        if (h->target == ENC_CTRL_FFT_MODE) {
            if (delta > 0)
                h->cur_fft_mode = (h->cur_fft_mode + 1) % 2;
            else
                h->cur_fft_mode = (h->cur_fft_mode + 1) % 2;
        }
    }

    // 按键检测
    uint8_t pin_state = HAL_GPIO_ReadPin(h->gpio_port, h->gpio_pin);
    uint32_t now = HAL_GetTick();

    if (pin_state == GPIO_PIN_RESET) {  // 按下
        if (!h->press_active) {
            h->press_active = 1;
            h->press_start = now;
        }
    } else {  // 松开
        if (h->press_active) {
            uint32_t dur = now - h->press_start;
            h->press_active = 0;

            if (dur >= LONG_PRESS_MS) {
                evt = ENC_EVT_LONG_PRESS;
                h->click_count = 0;
            } else {
                h->click_count++;
                if (h->click_count == 1) {
                    // 等待看是否有双击
                    h->press_start = now;  // 复用计时
                } else if (h->click_count >= 2) {
                    evt = ENC_EVT_DOUBLE_CLICK;
                    h->click_count = 0;
                }
            }
        }
    }

    // 双击超时检测
    if (h->click_count == 1 && !h->press_active) {
        if (now - h->press_start > CLICK_TIMEOUT) {
            evt = ENC_EVT_CLICK;
            h->click_count = 0;
        }
    }

    return evt;
}

// ENC_ApplyToDDS: 12项目不用，已注释
/*
void ENC_ApplyToDDS(ENC_Handle *h, DDS_Handle *dds) {
    switch (h->target) {
    case ENC_CTRL_FREQ:
        DDS_SetFreq(dds, h->cur_freq);
        break;
    case ENC_CTRL_AMP:
        DDS_SetAmp(dds, h->cur_amp);
        break;
    case ENC_CTRL_PHASE:
        DDS_SetPhase(dds, h->cur_phase);
        break;
    case ENC_CTRL_WAVE:
        DDS_SetWave(dds, h->cur_wave, dds->freq, dds->amp_vpp, dds->phase_deg);
        break;
    default: break;
    }
    h->cur_freq = dds->freq;
    h->cur_amp = dds->amp_vpp;
}
*/

void ENC_SetTarget(ENC_Handle *h, ENC_CtrlTarget_t target) {
    h->target = target;
}

ENC_CtrlTarget_t ENC_GetTarget(ENC_Handle *h) {
    return h->target;
}

void ENC_NextTarget(ENC_Handle *h) {
    h->target = (ENC_CtrlTarget_t)((h->target + 1) % 6);
}

uint8_t ENC_GetGear(ENC_Handle *h) {
    // 简化：根据总旋转量粗略判断速度
    // 实际应该用时间差，这里先用步数近似
    return 0;  // TODO: 实现连续档位速度检测
}

const char* ENC_TargetStr(ENC_CtrlTarget_t t) {
    switch (t) {
    case ENC_CTRL_FREQ:      return "FREQ";
    case ENC_CTRL_AMP:       return "AMP";
    case ENC_CTRL_PHASE:     return "PHA";
    case ENC_CTRL_WAVE:      return "WAV";
    case ENC_CTRL_PRESET:    return "PRE";
    case ENC_CTRL_FFT_MODE:  return "FFT";
    default:                 return "?";
    }
}
