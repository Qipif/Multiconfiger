#include "myfft.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define PI_F 3.1415926f
#define FFT_MAX_SIZE 1024

// ---- FFT句柄定义 ----
struct FFT_Handle {
    arm_cfft_instance_f32 cfft_fast;    // 256点实例
    arm_cfft_instance_f32 cfft_precise; // 1024点实例
    uint16_t fft_size;          // 当前使用的FFT点数
    float sample_rate;          // 采样率
    float adc_scale;            // ADC转电压系数
    FFT_Mode_t mode;            // 当前模式
    FFT_Window_t window;        // 当前窗函数
    float win_comp;             // 窗相干增益补偿因子

    // 预分配内存（按FFT_MAX_SIZE分配，切换模式不重新malloc）
    float fft_in[2 * FFT_MAX_SIZE];  // 复数输入
    float fft_in2[2 * FFT_MAX_SIZE]; // ch2复数输入（双通道相位差用）
    float fft_mag[FFT_MAX_SIZE];     // 幅度谱
    float sig_buf[FFT_MAX_SIZE];     // 信号缓冲
    float mag_backup[FFT_MAX_SIZE];  // 幅度谱备份（双信号分离用）

    // 防抖
    FFT_Wave_t last_type;
    uint8_t stable_cnt;
    FFT_Wave_t last_type2;
    uint8_t stable_cnt2;

    // 双通道相位差
    float phase_diff_f;
    float phase_offset;  // 硬件延迟补偿偏置(°)
};

// ---- 内部函数 ----
static void _apply_window(float *sig, uint16_t n, FFT_Window_t win);
static float _calc_win_comp(uint16_t n, FFT_Window_t win);
static uint16_t _find_peak(const float *mag, uint16_t start, uint16_t end);
static uint16_t _find_second_peak(const float *mag, uint16_t start, uint16_t end,
                                   uint16_t first_peak, uint16_t suppress);
static float _get_harmonic(const float *mag, uint16_t base_idx, uint8_t order, uint16_t n);
static FFT_Wave_t _identify_wave(const float *mag, uint16_t base_idx, uint16_t n);
static FFT_Wave_t _debounce(FFT_Wave_t cur, FFT_Wave_t *last, uint8_t *cnt);
static void _deinterleave(const uint16_t *src, float *ch1, float *ch2, uint16_t n, float scale);

// ---- 初始化 ----
FFT_Handle* MYFFT_Init(float sample_rate, float adc_vref, uint8_t adc_bits) {
    // 静态分配，不用malloc
    static FFT_Handle instance;
    FFT_Handle *h = &instance;

    h->sample_rate = sample_rate;
    h->adc_scale = adc_vref / ((1 << adc_bits) - 1);
    h->mode = FFT_MODE_PRECISE;
    h->window = FFT_WIN_HANNING;
    h->fft_size = FFT_MAX_SIZE;

    // 预初始化两个cfft实例
    arm_cfft_init_f32(&h->cfft_fast, 256);
    arm_cfft_init_f32(&h->cfft_precise, FFT_MAX_SIZE);

    // 算好窗补偿
    h->win_comp = _calc_win_comp(h->fft_size, h->window);

    // 防抖清零
    h->last_type = FFT_UNKNOWN;
    h->stable_cnt = 0;
    h->last_type2 = FFT_UNKNOWN;
    h->stable_cnt2 = 0;

    // 相位差
    h->phase_diff_f = 0.0f;
    h->phase_offset = 0.0f;

    return h;
}

void MYFFT_DeInit(FFT_Handle *h) {
    // 静态分配，不需要free
    (void)h;
}

// ---- 运行时配置 ----
void MYFFT_SetMode(FFT_Handle *h, FFT_Mode_t mode) {
    h->mode = mode;
    if (mode == FFT_MODE_FAST) {
        h->fft_size = 256;
        h->window = FFT_WIN_RECT;
    } else {
        h->fft_size = FFT_MAX_SIZE;
        h->window = FFT_WIN_HANNING;
    }
    h->win_comp = _calc_win_comp(h->fft_size, h->window);
}

void MYFFT_SetSampleRate(FFT_Handle *h, float fs) {
    h->sample_rate = fs;
}

void MYFFT_SetWindow(FFT_Handle *h, FFT_Window_t win) {
    h->window = win;
    h->win_comp = _calc_win_comp(h->fft_size, h->window);
}

// ---- 主处理 ----
void MYFFT_Process(FFT_Handle *h, const uint16_t *adc_buf, FFT_Result *result) {
    if (!h || !adc_buf || !result) return;

    uint32_t t0 = HAL_GetTick();
    uint16_t n = h->fft_size;
    float *sig = h->sig_buf;
    float *mag = h->fft_mag;
    float *fin = h->fft_in;

    // 1. 去直流 + ADC转电压
    float dc = 0;
    for (uint16_t i = 0; i < n; i++) dc += (float)adc_buf[i];
    dc /= n;
    for (uint16_t i = 0; i < n; i++) {
        sig[i] = ((float)adc_buf[i] - dc) * h->adc_scale;
    }

    // 2. 加窗
    _apply_window(sig, n, h->window);

    // 3. 构建复数输入
    for (uint16_t i = 0; i < n; i++) {
        fin[2 * i]     = sig[i];
        fin[2 * i + 1] = 0.0f;
    }

    // 4. 执行FFT
    if (n == 256) {
        arm_cfft_f32(&h->cfft_fast, fin, 0, 1);
    } else {
        arm_cfft_f32(&h->cfft_precise, fin, 0, 1);
    }

    // 5. 计算幅度谱
    arm_cmplx_mag_f32(fin, mag, n);

    // 6. 归一化 + 窗补偿
    mag[0] /= n;
    for (uint16_t i = 1; i < n; i++) {
        mag[i] = mag[i] * h->win_comp / (n / 2);
    }

    // 7. 找主峰（从bin=1开始，不跳过低频）
    uint16_t peak = _find_peak(mag, 1, n / 2 - 1);
    float base_amp = mag[peak];
    float freq = (float)peak * h->sample_rate / n;
    float phase = atan2f(fin[2 * peak + 1], fin[2 * peak]);

    // 8. 波形识别 + 防抖
    FFT_Wave_t raw_type = _identify_wave(mag, peak, n);
    FFT_Wave_t confirmed = _debounce(raw_type, &h->last_type, &h->stable_cnt);

    // 9. 填充结果
    result->frequency = freq;
    result->amplitude = base_amp;
    result->phase = phase;
    result->wave_type = confirmed;
    result->freq2 = 0;
    result->amp2 = 0;
    result->wave2 = FFT_UNKNOWN;

    result->process_ms = (float)(HAL_GetTick() - t0);
}

// ---- 双信号分离 ----
void MYFFT_ProcessDual(FFT_Handle *h, const uint16_t *adc_buf,
                        FFT_Result *r_low, FFT_Result *r_high) {
    if (!h || !adc_buf || !r_low || !r_high) return;

    uint32_t t0 = HAL_GetTick();
    uint16_t n = h->fft_size;
    float *mag = h->fft_mag;
    float *fin = h->fft_in;
    float *backup = h->mag_backup;

    // 先做一次完整的FFT处理
    MYFFT_Process(h, adc_buf, r_low);
    if (r_low->frequency == 0) {
        memset(r_high, 0, sizeof(FFT_Result));
        return;
    }

    // 备份幅度谱（_find_second_peak需要原始数据）
    memcpy(backup, mag, n / 2 * sizeof(float));

    // suppress宽度按分辨率挂钩: 至少5个bin
    uint16_t suppress = 5;
    uint16_t peak1 = (uint16_t)(r_low->frequency * n / h->sample_rate + 0.5f);

    // 在备份中屏蔽主峰后找第二峰
    uint16_t peak2 = _find_second_peak(backup, 1, n / 2 - 1, peak1, suppress);
    float amp2 = backup[peak2];

    // 第二峰有效性: 至少是主峰5%
    if (amp2 > r_low->amplitude * 0.05f && peak2 != peak1) {
        float freq2 = (float)peak2 * h->sample_rate / n;
        float phase2 = atan2f(fin[2 * peak2 + 1], fin[2 * peak2]);
        FFT_Wave_t raw2 = _identify_wave(mag, peak2, n);
        FFT_Wave_t confirmed2 = _debounce(raw2, &h->last_type2, &h->stable_cnt2);

        // 按频率高低分配
        if (r_low->frequency <= freq2) {
            r_high->frequency = freq2;
            r_high->amplitude = amp2;
            r_high->phase = phase2;
            r_high->wave_type = confirmed2;
        } else {
            *r_high = *r_low;
            r_low->frequency = freq2;
            r_low->amplitude = amp2;
            r_low->phase = phase2;
            r_low->wave_type = confirmed2;
            r_high->freq2 = 0;
            r_high->amp2 = 0;
            r_high->wave2 = FFT_UNKNOWN;
        }
        r_low->freq2 = r_high->frequency;
        r_low->amp2 = r_high->amplitude;
        r_low->wave2 = r_high->wave_type;
    } else {
        memset(r_high, 0, sizeof(FFT_Result));
    }

    // 更新耗时（包含双信号分离）
    r_low->process_ms = (float)(HAL_GetTick() - t0);
    r_high->process_ms = r_low->process_ms;
}

// ---- 内部函数实现 ----

static void _apply_window(float *sig, uint16_t n, FFT_Window_t win) {
    if (win == FFT_WIN_RECT) return;  // 矩形窗: 不加窗
    // 汉宁窗
    for (uint16_t i = 0; i < n; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * PI_F * i / (n - 1)));
        sig[i] *= w;
    }
}

// 计算窗相干增益补偿因子（Init时调一次）
static float _calc_win_comp(uint16_t n, FFT_Window_t win) {
    if (win == FFT_WIN_RECT) return 1.0f;  // 矩形窗不需要补偿
    float sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += 0.5f * (1.0f - cosf(2.0f * PI_F * i / (n - 1)));
    }
    return (float)n / sum;  // ≈ 2.0 for hanning
}

// 找最大峰
static uint16_t _find_peak(const float *mag, uint16_t start, uint16_t end) {
    uint16_t peak = start;
    float maxv = mag[start];
    for (uint16_t i = start + 1; i <= end; i++) {
        if (mag[i] > maxv) { maxv = mag[i]; peak = i; }
    }
    return peak;
}

// 找第二峰（屏蔽第一峰附近）
static uint16_t _find_second_peak(const float *mag, uint16_t start, uint16_t end,
                                   uint16_t first_peak, uint16_t suppress) {
    // 先把主峰附近清零（在备份数据上操作）
    float *m = (float *)mag;  // backup是可修改的
    int16_t lo = (first_peak > suppress) ? (first_peak - suppress) : start;
    uint16_t hi = (first_peak + suppress < end) ? (first_peak + suppress) : end;
    for (uint16_t i = lo; i <= hi; i++) {
        m[i] = 0;
    }
    return _find_peak(m, start, end);
}

// 获取谐波幅值（在峰值附近±2个bin搜最大值）
static float _get_harmonic(const float *mag, uint16_t base_idx, uint8_t order, uint16_t n) {
    uint16_t center = base_idx * order;
    if (center >= n / 2 || center < 1) return 0.0f;
    uint16_t s = (center > 2) ? (center - 2) : 1;
    uint16_t e = (center + 2 < n / 2) ? (center + 2) : (n / 2 - 1);
    return mag[_find_peak(mag, s, e)];
}

// 波形识别（3/5/7次谐波比判断，参考09版本）
static FFT_Wave_t _identify_wave(const float *mag, uint16_t base_idx, uint16_t n) {
    float base_amp = mag[base_idx];
    if (base_amp < 0.001f) return FFT_UNKNOWN;

    float h3 = _get_harmonic(mag, base_idx, 3, n);
    float h5 = _get_harmonic(mag, base_idx, 5, n);
    float h7 = _get_harmonic(mag, base_idx, 7, n);
    float r3 = h3 / base_amp;
    float r5 = h5 / base_amp;
    float r7 = h7 / base_amp;

    // 方波: 1/3, 1/5, 1/7
    const float sq_r3 = 1.0f / 3;
    const float sq_r5 = 1.0f / 5;
    const float sq_r7 = 1.0f / 7;
    const float tol = 0.35f;

    if (fabsf(r3 - sq_r3) < sq_r3 * tol &&
        fabsf(r5 - sq_r5) < sq_r5 * tol &&
        fabsf(r7 - sq_r7) < sq_r7 * tol) {
        return FFT_SQUARE;
    }

    // 三角波: 1/9
    const float tri_r3 = 1.0f / 9;
    if (fabsf(r3 - tri_r3) < tri_r3 * tol) {
        return FFT_TRIANGLE;
    }

    // 正弦波: 谐波很小
    if (r3 < 0.08f && r5 < 0.05f) {
        return FFT_SINE;
    }

    return FFT_UNKNOWN;
}

// 防抖: 连续3次相同才确认
static FFT_Wave_t _debounce(FFT_Wave_t cur, FFT_Wave_t *last, uint8_t *cnt) {
    if (cur == *last) {
        if (*cnt < 3) (*cnt)++;
    } else {
        *cnt = 0;
        *last = cur;
    }
    return (*cnt >= 3) ? cur : *last;
}

// ---- 工具函数 ----
const char* FFT_WaveStr(FFT_Wave_t w) {
    switch (w) {
    case FFT_SINE:     return "SIN";
    case FFT_SQUARE:   return "SQR";
    case FFT_TRIANGLE: return "TRI";
    default:           return "UNK";
    }
}

const char* FFT_ModeStr(FFT_Mode_t m) {
    switch (m) {
    case FFT_MODE_FAST:    return "FST";
    case FFT_MODE_PRECISE: return "PRC";
    default:               return "?";
    }
}

// ========================================================================
// 双通道相位差FFT（交错ADC数据: [ch1,ch2,ch1,ch2,...]）
// 固定256点+矩形窗，适合快速响应
// ========================================================================

#define PPHASE_LEN 256

static void _deinterleave(const uint16_t *src, float *ch1, float *ch2, uint16_t n, float scale) {
    float dc1 = 0, dc2 = 0;
    for (uint16_t i = 0; i < n; i++) {
        dc1 += src[2 * i];
        dc2 += src[2 * i + 1];
    }
    dc1 /= n;
    dc2 /= n;
    for (uint16_t i = 0; i < n; i++) {
        ch1[i] = ((float)src[2 * i]     - dc1) * scale;
        ch2[i] = ((float)src[2 * i + 1] - dc2) * scale;
    }
}

void MYFFT_ProcessPhase(FFT_Handle *h, const uint16_t *dual_adc_buf, FFT_Result *result) {
    if (!h || !dual_adc_buf || !result) return;

    uint16_t n = PPHASE_LEN;
    float scale = h->adc_scale;

    static float ch2_buf[PPHASE_LEN];  // 独立ch2 buffer

    // 1. 去交错 + 去直流
    _deinterleave(dual_adc_buf, h->sig_buf, ch2_buf, n, scale);

    // 2. 构建复数输入
    for (uint16_t i = 0; i < n; i++) {
        h->fft_in[2 * i]     = h->sig_buf[i];
        h->fft_in[2 * i + 1] = 0.0f;
        h->fft_in2[2 * i]     = ch2_buf[i];
        h->fft_in2[2 * i + 1] = 0.0f;
    }

    // 3. FFT
    arm_cfft_f32(&h->cfft_fast, h->fft_in, 0, 1);
    arm_cfft_f32(&h->cfft_fast, h->fft_in2, 0, 1);

    // 4. 找主频bin（首次锁定，信号丢失时重扫）
    static uint16_t peak = 0;
    float re_peak = h->fft_in[2 * peak];
    float im_peak = h->fft_in[2 * peak + 1];
    float mag_peak = re_peak * re_peak + im_peak * im_peak;

    if (peak == 0 || mag_peak < 0.001f) {
        float max_mag = 0;
        peak = 1;
        for (uint16_t i = 1; i < n / 2; i++) {
            float re = h->fft_in[2 * i];
            float im = h->fft_in[2 * i + 1];
            float mag = re * re + im * im;
            if (mag > max_mag) {
                max_mag = mag;
                peak = i;
            }
        }
    }

    // 5. 幅值 & 频率
    float re1 = h->fft_in[2 * peak];
    float im1 = h->fft_in[2 * peak + 1];
    float re2 = h->fft_in2[2 * peak];
    float im2 = h->fft_in2[2 * peak + 1];

    float mag = sqrtf(re1 * re1 + im1 * im1);
    result->frequency = (float)peak * h->sample_rate / n;
    result->amplitude = mag * 2.0f / n;

    // 6. 相位差 = angle(X2 * conj(X1))
    float re = re2 * re1 + im2 * im1;
    float im = im2 * re1 - re2 * im1;
    float dphi = atan2f(im, re) * 180.0f / PI_F;

    if (dphi > 180.0f)  dphi -= 360.0f;
    if (dphi < -180.0f) dphi += 360.0f;

    // 7. 一阶滤波 + 偏置补偿
    h->phase_diff_f = h->phase_diff_f * 0.7f + dphi * 0.3f;
    result->phase_diff = h->phase_diff_f - h->phase_offset;
    result->phase = atan2f(im1, re1);

    // 8. 波形识别
    for (uint16_t i = 1; i < n / 2; i++) {
        float re = h->fft_in[2 * i];
        float im = h->fft_in[2 * i + 1];
        h->fft_mag[i] = sqrtf(re * re + im * im);
    }
    FFT_Wave_t raw = _identify_wave(h->fft_mag, peak, n);
    result->wave_type = _debounce(raw, &h->last_type, &h->stable_cnt);
    result->freq2 = 0;
    result->amp2 = 0;
    result->wave2 = FFT_UNKNOWN;
}

void MYFFT_SetPhaseOffset(FFT_Handle *h, float offset_deg) {
    h->phase_offset = offset_deg;
}

const float* MYFFT_GetMagBuffer(FFT_Handle *h) {
    return h ? h->fft_mag : NULL;
}

