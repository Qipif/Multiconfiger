#include "myfft.h"
#include <math.h>
#include <stdlib.h>

#define M_PI_F 3.1415926f

struct MYFFT_Handle {
    arm_cfft_instance_f32 cfft_inst;
    uint16_t fft_size;
    float sample_rate;
    float adc_scale;
    float *fft_in;      // 长度 2*fft_size
    float *fft_mag;     // 长度 fft_size
    float *sig_buf;     // 长度 fft_size

    WaveType last_type;
    uint8_t  confirm_cnt;
    WaveType confirmed_type;
    float    last_freq;

    float *result_mag;  // 指向结果结构体中的 mag 数组，由外部传入
};

static void apply_hanning_window(float *buf, uint16_t len);
static uint16_t find_peak_index(float *mag, uint16_t start, uint16_t end);
static float get_harmonic_mag(MYFFT_Handle *h, uint16_t base_idx, uint8_t order);
static WaveType identify_waveform(MYFFT_Handle *h, uint16_t base_idx);

MYFFT_Handle* MYFFT_Init(uint16_t fft_size, float sample_rate,
                         float adc_vref, uint8_t adc_bits)
{
    MYFFT_Handle *h = (MYFFT_Handle*)malloc(sizeof(MYFFT_Handle));
    if (h == NULL) return NULL;

    h->fft_size = fft_size;
    h->sample_rate = sample_rate;
    h->adc_scale = adc_vref / (float)((1 << adc_bits) - 1);

    h->fft_in  = (float*)malloc(2 * fft_size * sizeof(float));
    h->fft_mag = (float*)malloc(fft_size * sizeof(float));
    h->sig_buf = (float*)malloc(fft_size * sizeof(float));

    if (!h->fft_in || !h->fft_mag || !h->sig_buf) {
        free(h->fft_in); free(h->fft_mag); free(h->sig_buf);
        free(h);
        return NULL;
    }

    arm_cfft_init_f32(&h->cfft_inst, fft_size);

    h->last_type = WAVE_UNKNOWN;
    h->confirm_cnt = 0;
    h->confirmed_type = WAVE_UNKNOWN;
    h->last_freq = 0.0f;
    h->result_mag = NULL;

    return h;
}

void MYFFT_Process(MYFFT_Handle *h, const uint16_t *adc_buf, MYFFT_Result *result)
{
    if (!h || !adc_buf || !result) return;

    uint16_t n = h->fft_size;
    float *sig = h->sig_buf;
    float *mag = h->fft_mag;
    float *fft_in = h->fft_in;

    // 1. 直流分量
    float dc = 0.0f;
    for (uint16_t i = 0; i < n; i++) dc += (float)adc_buf[i];
    dc /= n;

    // 2. 去直流并加窗
    for (uint16_t i = 0; i < n; i++) {
        sig[i] = ((float)adc_buf[i] - dc) * h->adc_scale;
    }
    apply_hanning_window(sig, n);

    // 3. 构建复数输入
    for (uint16_t i = 0; i < n; i++) {
        fft_in[2 * i]     = sig[i];
        fft_in[2 * i + 1] = 0.0f;
    }

    // 4. 执行 FFT
    arm_cfft_f32(&h->cfft_inst, fft_in, 0, 1);

    // 5. 计算幅度谱
    arm_cmplx_mag_f32(fft_in, mag, n);

    // 6. 归一化并补偿汉宁窗
    // 计算汉宁窗相干增益补偿因子（一次即可，可放在初始化中优化）
    float win_coherent_gain = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        win_coherent_gain += 0.5f * (1.0f - cosf(2.0f * M_PI_F * i / (n - 1)));
    }
    win_coherent_gain /= n;
    float comp = 1.0f / win_coherent_gain;   // ≈ 2.0

    mag[0] /= n;   // 直流分量
    for (uint16_t i = 1; i < n; i++) {
        mag[i] = mag[i] * comp / (n / 2);
    }

    // 7. 查找基频峰值
    uint16_t peak_idx = find_peak_index(mag, 2, n / 2);
    float base_amp = mag[peak_idx];
    float freq = (float)peak_idx * h->sample_rate / n;
    float phase = atan2f(fft_in[2 * peak_idx + 1], fft_in[2 * peak_idx]);

    // 8. 波形识别（省略防抖逻辑，保持简洁）
    WaveType raw_type = identify_waveform(h, peak_idx);
    if (raw_type == h->last_type) {
        h->confirm_cnt++;
        if (h->confirm_cnt >= 3) {
            h->confirmed_type = raw_type;
            h->last_freq = freq;
        }
    } else {
        h->confirm_cnt = 0;
        h->last_type = raw_type;
    }

    // 9. 填充结果
    result->wave_type = h->confirmed_type;
    result->frequency = h->last_freq;
    result->amplitude = base_amp;
    result->phase = phase;
    result->snr = 0.0f;

    if (result->mag != NULL && result->mag_len >= n/2) {
        for (uint16_t i = 0; i < n/2; i++) {
            result->mag[i] = mag[i];
        }
    }
}

void MYFFT_DeInit(MYFFT_Handle *h) {
    if (h) {
        free(h->fft_in);
        free(h->fft_mag);
        free(h->sig_buf);
        free(h);
    }
}

/* ---------- 内部函数 ---------- */
static void apply_hanning_window(float *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI_F * i / (len - 1)));
        buf[i] *= w;
    }
}

static uint16_t find_peak_index(float *mag, uint16_t start, uint16_t end) {
    uint16_t peak = start;
    float max_val = mag[start];
    for (uint16_t i = start + 1; i <= end; i++) {
        if (mag[i] > max_val) {
            max_val = mag[i];
            peak = i;
        }
    }
    return peak;
}

static float get_harmonic_mag(MYFFT_Handle *h, uint16_t base_idx, uint8_t order) {
    uint16_t n = h->fft_size;
    uint16_t center = base_idx * order;
    if (center < 3) return 0.0f;
    uint16_t start = (center > 2) ? (center - 2) : 2;
    uint16_t end   = (center + 2 < n / 2) ? (center + 2) : (n / 2 - 1);
    uint16_t peak = find_peak_index(h->fft_mag, start, end);
    return h->fft_mag[peak];
}

static WaveType identify_waveform(MYFFT_Handle *h, uint16_t base_idx) {
    float *mag = h->fft_mag;
    float base_amp = mag[base_idx];
    if (base_amp < 0.001f) return WAVE_UNKNOWN;

    float h3 = get_harmonic_mag(h, base_idx, 3);
    float h5 = get_harmonic_mag(h, base_idx, 5);
    float h7 = get_harmonic_mag(h, base_idx, 7);
    float r3 = h3 / base_amp;
    float r5 = h5 / base_amp;
    float r7 = h7 / base_amp;

    const float tri_r3 = 1.0f / 9;
    const float sq_r3  = 1.0f / 3;
    const float sq_r5  = 1.0f / 5;
    const float sq_r7  = 1.0f / 7;
    const float tolerance = 0.35f;

    if (fabsf(r3 - sq_r3) < sq_r3 * tolerance &&
        fabsf(r5 - sq_r5) < sq_r5 * tolerance &&
        fabsf(r7 - sq_r7) < sq_r7 * tolerance) {
        return WAVE_SQUARE;
    }
    if (fabsf(r3 - tri_r3) < tri_r3 * tolerance) {
        return WAVE_TRIANGLE;
    }
    if (r3 < 0.08f && r5 < 0.05f) {
        return WAVE_SINE;
    }
    return WAVE_UNKNOWN;
}

// --------------------- myfft.c 新增内容 ---------------------

// 辅助函数：将波形类型转为字符串
const char* WaveType_ToString(WaveType type) {
    switch(type) {
        case WAVE_SINE:     return "SIN";
        case WAVE_TRIANGLE: return "TRI";
        case WAVE_SQUARE:   return "SQR";
        default:            return "UNK";
    }
}

// 辅助函数：在指定范围内寻找第二峰值（排除第一峰值附近的泄露）
static uint16_t find_second_peak_index(float *mag, uint16_t start, uint16_t end, uint16_t first_peak) {
    // 定义屏蔽宽度，防止把主峰的频谱泄露当成第二主频 (假设频率分辨率足够分辨5kHz)
    uint16_t suppress_width = 5;
    uint16_t peak = start;
    float max_val = -1.0f;

    for (uint16_t i = start; i <= end; i++) {
        // 跳过第一主峰附近的频点
        if (i >= (first_peak > suppress_width ? first_peak - suppress_width : start) &&
            i <= (first_peak + suppress_width < end ? first_peak + suppress_width : end)) {
            continue;
        }
        if (mag[i] > max_val) {
            max_val = mag[i];
            peak = i;
        }
    }
    return peak;
}

/**
 * @brief 处理混合信号，提取两个信号的参数
 */
void MYFFT_ProcessDual(MYFFT_Handle *h, const uint16_t *adc_buf, MYFFT_Result *res1, MYFFT_Result *res2) {
    if (!h || !adc_buf || !res1 || !res2) return;

    uint16_t n = h->fft_size;
    float *sig = h->sig_buf;
    float *mag = h->fft_mag;
    float *fft_in = h->fft_in;

    // 1 & 2. 去直流并加窗 (复用原有逻辑)
    float dc = 0.0f;
    for (uint16_t i = 0; i < n; i++) dc += (float)adc_buf[i];
    dc /= n;
    for (uint16_t i = 0; i < n; i++) {
        sig[i] = ((float)adc_buf[i] - dc) * h->adc_scale;
    }
    apply_hanning_window(sig, n);

    // 3 & 4. 构建复数并执行 FFT
    for (uint16_t i = 0; i < n; i++) {
        fft_in[2 * i]     = sig[i];
        fft_in[2 * i + 1] = 0.0f;
    }
    arm_cfft_f32(&h->cfft_inst, fft_in, 0, 1);

    // 5 & 6. 计算幅度谱并归一化
    arm_cmplx_mag_f32(fft_in, mag, n);
    float win_coherent_gain = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        win_coherent_gain += 0.5f * (1.0f - cosf(2.0f * M_PI_F * i / (n - 1)));
    }
    win_coherent_gain /= n;
    float comp = 1.0f / win_coherent_gain;
    mag[0] /= n;
    for (uint16_t i = 1; i < n; i++) {
        mag[i] = mag[i] * comp / (n / 2);
    }

    // 7. 寻找两个主频
    uint16_t search_end = n / 2;
    uint16_t peak1_idx = find_peak_index(mag, 2, search_end);
    uint16_t peak2_idx = find_second_peak_index(mag, 2, search_end, peak1_idx);

    // 按频率大小排序：保证 res1 是低频(fA)，res2 是高频
    uint16_t idx_low  = (peak1_idx < peak2_idx) ? peak1_idx : peak2_idx;
    uint16_t idx_high = (peak1_idx < peak2_idx) ? peak2_idx : peak1_idx;

    // 8. 提取低频信号 (res1) 参数
    float freq_low = (float)idx_low * h->sample_rate / n;
    float amp_low  = mag[idx_low];
    float phase_low = atan2f(fft_in[2 * idx_low + 1], fft_in[2 * idx_low]);
    WaveType type_low = (amp_low > 0.001f) ? identify_waveform(h, idx_low) : WAVE_UNKNOWN;

    // 9. 提取高频信号 (res2) 参数
    float freq_high = (float)idx_high * h->sample_rate / n;
    float amp_high  = mag[idx_high];
    float phase_high = atan2f(fft_in[2 * idx_high + 1], fft_in[2 * idx_high]);
    WaveType type_high = (amp_high > 0.001f) ? identify_waveform(h, idx_high) : WAVE_UNKNOWN;

    // 10. 填充结果结构体
    res1->wave_type = type_low;
    res1->frequency = freq_low;
    res1->amplitude = amp_low;
    res1->phase = phase_low;
    res1->snr = 0.0f;

    res2->wave_type = type_high;
    res2->frequency = freq_high;
    res2->amplitude = amp_high;
    res2->phase = phase_high;
    res2->snr = 0.0f;
}
