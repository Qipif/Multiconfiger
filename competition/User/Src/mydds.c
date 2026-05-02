#include "mydds.h"
#include <math.h>
#include <string.h>

// 256点正弦表(12位,中心2048)
static const uint16_t sine256[256] = {
    2048,2098,2148,2199,2249,2299,2348,2398,
    2447,2497,2545,2594,2642,2690,2738,2785,
    2831,2878,2923,2968,3013,3057,3100,3143,
    3185,3227,3267,3307,3347,3385,3423,3459,
    3495,3531,3565,3598,3630,3662,3692,3722,
    3750,3777,3804,3829,3853,3876,3898,3919,
    3939,3958,3975,3992,4007,4021,4034,4045,
    4056,4065,4073,4080,4085,4089,4093,4094,
    4095,4094,4093,4089,4085,4080,4073,4065,
    4056,4045,4034,4021,4007,3992,3975,3958,
    3939,3919,3898,3876,3853,3829,3804,3777,
    3750,3722,3692,3662,3630,3598,3565,3531,
    3495,3459,3423,3385,3347,3307,3267,3227,
    3185,3143,3100,3057,3013,2968,2923,2878,
    2831,2785,2738,2690,2642,2594,2545,2497,
    2447,2398,2348,2299,2249,2199,2148,2098,
    2048,1998,1948,1897,1847,1797,1748,1698,
    1649,1599,1551,1502,1454,1406,1358,1311,
    1265,1218,1173,1128,1083,1039,996,953,
    911,869,829,789,749,711,673,637,
    601,565,531,498,466,434,404,374,
    346,319,292,267,243,220,198,177,
    157,138,121,104,89,75,62,51,
    40,31,23,16,11,7,3,2,
    1,2,3,7,11,16,23,31,
    40,51,62,75,89,104,121,138,
    157,177,198,220,243,267,292,319,
    346,374,404,434,466,498,531,565,
    601,637,673,711,749,789,829,869,
    911,953,996,1039,1083,1128,1173,1218,
    1265,1311,1358,1406,1454,1502,1551,1599,
    1649,1698,1748,1797,1847,1897,1948,1998
};

#define PI_F 3.1415926f
#define DAC_MID 2048
#define DAC_MAX 4095

// 获取定时器时钟
static uint32_t _tim_clk(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 || htim->Instance == TIM8 ||
        htim->Instance == TIM15 || htim->Instance == TIM16 ||
        htim->Instance == TIM17)
        return HAL_RCC_GetPCLK2Freq() * 2;
    return HAL_RCC_GetPCLK1Freq() * 2;
}

// 默认配置
static const DDS_Config _default_cfg = {
    .algo = DDS_ALGO_FAST,
    .wave = DDS_SINE,
    .table_size = 256,
    .target_rate = 10000000UL,
    .auto_switch_freq = 50000.0f,
    .enable_interp = 1
};

// 根据频率自动选择算法
static DDS_Algo_t _auto_algo(DDS_Handle *h, float freq) {
    if (h->cfg.auto_switch_freq > 0 && freq >= h->cfg.auto_switch_freq)
        return DDS_ALGO_PRECISE;
    return h->cfg.algo;
}

// ---- 高精度算法：动态点数+相位累加+线性插值 ----
static void _update_precise(DDS_Handle *h) {
    if (h->freq <= 0.0f) return;

    uint32_t rate = h->cfg.target_rate;
    uint32_t pts = rate / (uint32_t)h->freq;
    if (pts < 10) pts = 10;
    if (pts > 512) pts = 512;
    h->buf_len = pts;

    uint32_t clk = _tim_clk(h->htim);
    uint32_t sr = (uint32_t)(h->freq * pts);
    uint32_t arr = (uint32_t)((float)clk / sr + 0.5f);
    if (arr < 2) arr = 2;
    if (arr > 65535) arr = 65535;

    HAL_TIM_Base_Stop(h->htim);
    __HAL_TIM_SET_AUTORELOAD(h->htim, arr - 1);
    __HAL_TIM_SET_COUNTER(h->htim, 0);

    uint32_t step = (uint32_t)(h->freq * 4294967296.0f / sr);
    float scale = (h->amp_vpp / 2.0f) / 1.65f;
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;

    uint16_t *buf = (h->buf_idx == 0) ? h->buf0 : h->buf1;
    uint32_t phase = h->phase_acc;

    for (uint32_t i = 0; i < pts; i++) {
        int32_t val;
        switch (h->wave) {
        case DDS_SINE:
        default: {
            uint32_t idx = (phase >> 24) & 0xFF;
            uint32_t nxt = (idx + 1) & 0xFF;
            uint32_t frac = (phase >> 8) & 0xFFFF;
            int32_t v0 = sine256[idx], v1 = sine256[nxt];
            val = v0 + ((v1 - v0) * (int32_t)frac) / 65536;
            break;
        }
        case DDS_TRIANGLE: {
            uint32_t tp = (phase >> 22) & 0x3FF;
            val = (tp < 512) ? (tp * 8) : ((1023 - tp) * 8);
            break;
        }
        case DDS_SQUARE:
            val = (phase & 0x80000000) ? 0 : 4095;
            break;
        }
        int32_t s = (int32_t)((val - DAC_MID) * scale) + DAC_MID;
        if (s > DAC_MAX) s = DAC_MAX;
        if (s < 0) s = 0;
        buf[i] = (uint16_t)s;
        phase += step;
    }
    h->phase_acc = phase;
    h->active_buf = buf;

    SCB_CleanDCache_by_Addr((uint32_t*)buf, pts * sizeof(uint16_t));
    HAL_DAC_Stop_DMA(h->hdac, h->dac_channel);
    HAL_DAC_Start_DMA(h->hdac, h->dac_channel,
                      (uint32_t*)buf, pts, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(h->htim);
    h->buf_idx = 1 - h->buf_idx;
}

// ---- 快速查表算法：256点预计算表+DMA循环 ----
static void _update_fast(DDS_Handle *h) {
    if (h->freq <= 0.0f) return;

    uint16_t pts = h->cfg.table_size;
    if (pts > 256) pts = 256;
    h->buf_len = pts;

    // 定时器配置
    uint32_t clk = _tim_clk(h->htim);
    uint32_t sr = (uint32_t)(h->freq * pts);
    uint32_t arr = (uint32_t)((float)clk / sr + 0.5f);
    if (arr < 2) arr = 2;
    if (arr > 65535) arr = 65535;

    HAL_TIM_Base_Stop(h->htim);
    __HAL_TIM_SET_AUTORELOAD(h->htim, arr - 1);
    __HAL_TIM_SET_COUNTER(h->htim, 0);

    // 相位偏移(查表起始点)
    uint16_t phase_offset = (uint16_t)(h->phase_deg / 360.0f * pts);
    // 幅度缩放
    float scale = (h->amp_vpp / 2.0f) / 1.65f;
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;

    uint16_t *buf = (h->buf_idx == 0) ? h->buf0 : h->buf1;

    for (uint16_t i = 0; i < pts; i++) {
        uint16_t ti = (i + phase_offset) % pts;
        int32_t val;
        switch (h->wave) {
        case DDS_SINE:
        default:
            val = sine256[ti];
            break;
        case DDS_TRIANGLE: {
            float p = (float)ti / pts;
            float tri = (p < 0.5f) ? (p * 2.0f) : (2.0f - p * 2.0f);
            val = (int32_t)(DAC_MID + (tri - 0.5f) * 2.0f * DAC_MID);
            break;
        }
        case DDS_SQUARE:
            val = (ti < pts / 2) ? DAC_MAX : 0;
            break;
        }
        int32_t s = (int32_t)((val - DAC_MID) * scale) + DAC_MID;
        if (s > DAC_MAX) s = DAC_MAX;
        if (s < 0) s = 0;
        buf[i] = (uint16_t)s;
    }
    h->active_buf = buf;

    SCB_CleanDCache_by_Addr((uint32_t*)buf, pts * sizeof(uint16_t));
    HAL_DAC_Stop_DMA(h->hdac, h->dac_channel);
    HAL_DAC_Start_DMA(h->hdac, h->dac_channel,
                      (uint32_t*)buf, pts, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(h->htim);
    h->buf_idx = 1 - h->buf_idx;
}

// ---- 公共API ----

void DDS_Init(DDS_Handle *h, TIM_HandleTypeDef *htim,
                DAC_HandleTypeDef *hdac, uint32_t ch) {
    DDS_InitWithConfig(h, &_default_cfg, htim, hdac, ch);
}

void DDS_InitWithConfig(DDS_Handle *h, const DDS_Config *cfg,
                           TIM_HandleTypeDef *htim,
                           DAC_HandleTypeDef *hdac, uint32_t ch) {
    memset(h, 0, sizeof(DDS_Handle));
    h->htim = htim;
    h->hdac = hdac;
    h->dac_channel = ch;
    h->cfg = *cfg;
    h->wave = cfg->wave;
    h->freq = 1000.0f;
    h->amp_vpp = 2.0f;
    h->phase_deg = 0.0f;
    h->phase_acc = 0;
    h->buf_idx = 0;
    h->active_buf = h->buf0;
}

void DDS_SetWave(DDS_Handle *h, DDS_Wave_t wave,
                   float freq, float amp_vpp, float phase_deg) {
    h->wave = wave;
    h->freq = freq;
    h->amp_vpp = amp_vpp;
    h->phase_deg = phase_deg;
    h->phase_acc = (uint32_t)(phase_deg / 360.0f * 4294967296.0f);

    DDS_Algo_t algo = _auto_algo(h, freq);
    if (algo == DDS_ALGO_PRECISE)
        _update_precise(h);
    else
        _update_fast(h);
}

// 独立调频：只改定时器ARR（最快路径）
void DDS_SetFreq(DDS_Handle *h, float freq) {
    if (freq <= 0.0f) return;
    h->freq = freq;

    DDS_Algo_t algo = _auto_algo(h, freq);
    if (algo != h->cfg.algo && h->cfg.auto_switch_freq > 0) {
        // 算法切换需要重生成
        if (algo == DDS_ALGO_PRECISE)
            _update_precise(h);
        else
            _update_fast(h);
        return;
    }

    // 快速路径：只改ARR
    uint32_t pts = h->buf_len;
    if (pts == 0) pts = h->cfg.table_size;
    uint32_t clk = _tim_clk(h->htim);
    uint32_t sr = (uint32_t)(freq * pts);
    uint32_t arr = (uint32_t)((float)clk / sr + 0.5f);
    if (arr < 2) arr = 2;
    if (arr > 65535) arr = 65535;
    __HAL_TIM_SET_AUTORELOAD(h->htim, arr - 1);
}

// 独立调幅：重新缩放当前buf
void DDS_SetAmp(DDS_Handle *h, float amp_vpp) {
    h->amp_vpp = amp_vpp;
    float scale = (amp_vpp / 2.0f) / 1.65f;
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;

    uint16_t *buf = h->active_buf;
    uint32_t len = h->buf_len;
    if (!buf || len == 0) return;

    // 从原始波表重新缩放
    uint16_t pts = (len > 256) ? 256 : len;
    for (uint32_t i = 0; i < len; i++) {
        uint16_t ti = i % pts;
        int32_t val;
        switch (h->wave) {
        case DDS_SINE:
        default:
            val = sine256[ti]; break;
        case DDS_TRIANGLE: {
            float p = (float)ti / pts;
            float tri = (p < 0.5f) ? (p * 2.0f) : (2.0f - p * 2.0f);
            val = (int32_t)(DAC_MID + (tri - 0.5f) * 2.0f * DAC_MID);
            break;
        }
        case DDS_SQUARE:
            val = (ti < pts / 2) ? DAC_MAX : 0; break;
        }
        int32_t s = (int32_t)((val - DAC_MID) * scale) + DAC_MID;
        if (s > DAC_MAX) s = DAC_MAX;
        if (s < 0) s = 0;
        buf[i] = (uint16_t)s;
    }
    SCB_CleanDCache_by_Addr((uint32_t*)buf, len * sizeof(uint16_t));
}

// 独立调相：重新生成（相位影响查表起始点）
void DDS_SetPhase(DDS_Handle *h, float phase_deg) {
    h->phase_deg = phase_deg;
    h->phase_acc = (uint32_t)(phase_deg / 360.0f * 4294967296.0f);
    DDS_Algo_t algo = _auto_algo(h, h->freq);
    if (algo == DDS_ALGO_PRECISE)
        _update_precise(h);
    else
        _update_fast(h);
}

// 通用调参
void DDS_SetParam(DDS_Handle *h, DDS_Param_t param, float val) {
    switch (param) {
    case DDS_PARAM_FREQ:  DDS_SetFreq(h, val); break;
    case DDS_PARAM_AMP:   DDS_SetAmp(h, val); break;
    case DDS_PARAM_PHASE: DDS_SetPhase(h, val); break;
    }
}

void DDS_SetAlgo(DDS_Handle *h, DDS_Algo_t algo) {
    h->cfg.algo = algo;
    if (algo == DDS_ALGO_PRECISE)
        _update_precise(h);
    else
        _update_fast(h);
}

void DDS_ApplyConfig(DDS_Handle *h, const DDS_Config *cfg) {
    h->cfg = *cfg;
    h->wave = cfg->wave;
    DDS_SetWave(h, h->wave, h->freq, h->amp_vpp, h->phase_deg);
}

void DDS_GetConfig(DDS_Handle *h, DDS_Config *cfg) {
    *cfg = h->cfg;
}

void DDS_Start(DDS_Handle *h) {
    DDS_SetWave(h, h->wave, h->freq, h->amp_vpp, h->phase_deg);
}

void DDS_Stop(DDS_Handle *h) {
    HAL_DAC_Stop_DMA(h->hdac, h->dac_channel);
    HAL_TIM_Base_Stop(h->htim);
}

const char* DDS_AlgoStr(DDS_Algo_t a) {
    return (a == DDS_ALGO_PRECISE) ? "PREC" : "FAST";
}

const char* DDS_WaveStr(DDS_Wave_t w) {
    switch (w) {
    case DDS_SINE:     return "SIN";
    case DDS_TRIANGLE: return "TRI";
    case DDS_SQUARE:   return "SQR";
    default:              return "UNK";
    }
}

// ========================================================================
// 实时DDS（RT）- 轻量化版
// 采样率=200kHz，每半帧64点=320us刷新
// ISR全整数，无浮点，无DCache操作
// ========================================================================

#define DDS_RT_SAMPLE_RATE 200000UL  // 200kHz（原来是1MHz）
#define DDS_RT_Q12_SCALE   2048     // Q12定点比例因子

// amp_vpp → Q12（Init/SetAmp时算好存到amp_scale_q12）
// 输出: s = ((val - 2048) * scale_q12 >> 11) + 2048
static inline uint16_t _rt_sin_q12(uint32_t phase, uint16_t scale_q12) {
    uint16_t val = sine256[(phase >> 24) & 0xFF];
    int32_t d = (int32_t)val - DAC_MID;
    int32_t s = (d * scale_q12) >> 11;  // Q12右移11=Q0
    s += DAC_MID;
    if (s > DAC_MAX) s = DAC_MAX;
    if (s < 0) s = 0;
    return (uint16_t)s;
}

// 填半帧（DMA半传输完成回调中调用）- ISR-safe
void DDS_RT_FillHalf(DDS_RT_Handle *h) {
    uint32_t half = DDS_RT_BUF_LEN / 2;
    uint32_t p1 = h->phase_acc1;
    uint32_t p2 = h->phase_acc2 + h->phase_off2;  // phase_off2即时生效
    uint32_t off2 = h->phase_off2;

    for (uint32_t i = 0; i < half; i++) {
        // 全整数，无DCache，纯查表+定点乘
        h->buf1[i] = _rt_sin_q12(p1, h->amp1_scale_q12);
        h->buf2[i] = _rt_sin_q12(p2, h->amp2_scale_q12);
        p1 += h->phase_step1;
        p2 += h->phase_step2;
    }
    h->phase_acc1 = p1;
    h->phase_acc2 = p2 - off2;  // 存裸值
}

// 填下半帧（DMA传输完成回调中调用）- ISR-safe
void DDS_RT_FillOther(DDS_RT_Handle *h) {
    uint32_t half = DDS_RT_BUF_LEN / 2;
    uint32_t p1 = h->phase_acc1;
    uint32_t p2 = h->phase_acc2 + h->phase_off2;
    uint32_t off2 = h->phase_off2;

    for (uint32_t i = 0; i < half; i++) {
        h->buf1[half + i] = _rt_sin_q12(p1, h->amp1_scale_q12);
        h->buf2[half + i] = _rt_sin_q12(p2, h->amp2_scale_q12);
        p1 += h->phase_step1;
        p2 += h->phase_step2;
    }
    h->phase_acc1 = p1;
    h->phase_acc2 = p2 - off2;
}

void DDS_RT_Init(DDS_RT_Handle *h, TIM_HandleTypeDef *htim, DAC_HandleTypeDef *hdac,
                 float freq, float amp_vpp, float phase2_deg) {
    memset(h, 0, sizeof(DDS_RT_Handle));
    h->htim = htim;
    h->hdac = hdac;
    h->wave1 = h->wave2 = DDS_SINE;
    h->freq1 = h->freq2 = freq;
    h->phase1_deg = 0.0f;
    h->phase2_deg = phase2_deg;

    // Q12幅度（2.0Vpp → scale=1245）
    uint32_t sc = (uint32_t)(amp_vpp * DDS_RT_Q12_SCALE * 1000.0f / 3300.0f);
    if (sc > DDS_RT_Q12_SCALE) sc = DDS_RT_Q12_SCALE;
    h->amp1_scale_q12 = (uint16_t)sc;
    h->amp2_scale_q12 = (uint16_t)sc;

    // 相位偏移Q32
    float pn2 = fmodf(phase2_deg, 360.0f);
    if (pn2 < 0) pn2 += 360.0f;
    h->phase_off2 = (uint32_t)(pn2 / 360.0f * 4294967296.0f);

    // TIM4设为200kHz
    uint32_t clk = _tim_clk(htim);
    uint32_t arr = clk / DDS_RT_SAMPLE_RATE;
    if (arr < 2) arr = 2;
    if (arr > 65535) arr = 65535;
    __HAL_TIM_SET_AUTORELOAD(htim, arr - 1);
    __HAL_TIM_SET_COUNTER(htim, 0);

    // 相位步进（Q32 / 200kHz）
    h->phase_step1 = (uint32_t)(freq * 4294967296.0f / DDS_RT_SAMPLE_RATE);
    h->phase_step2 = h->phase_step1;

    // 预填充buf（Init时做一次DCache）
    uint32_t p1 = 0;
    uint32_t p2 = h->phase_off2;
    uint32_t off2 = h->phase_off2;
    for (uint32_t i = 0; i < DDS_RT_BUF_LEN; i++) {
        h->buf1[i] = _rt_sin_q12(p1, h->amp1_scale_q12);
        h->buf2[i] = _rt_sin_q12(p2, h->amp2_scale_q12);
        p1 += h->phase_step1;
        p2 += h->phase_step2;
    }
    h->phase_acc1 = p1;
    h->phase_acc2 = p2 - off2;

    SCB_CleanDCache_by_Addr((uint32_t*)h->buf1, DDS_RT_BUF_LEN * sizeof(uint16_t));
    SCB_CleanDCache_by_Addr((uint32_t*)h->buf2, DDS_RT_BUF_LEN * sizeof(uint16_t));

    // 启动DMA（双通道独立，CIRCULAR，TIM4同触发）
    HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)h->buf1, DDS_RT_BUF_LEN, DAC_ALIGN_12B_R);
    HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_2, (uint32_t*)h->buf2, DDS_RT_BUF_LEN, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(htim);
}

void DDS_RT_SetFreq(DDS_RT_Handle *h, float freq) {
    h->freq1 = freq;
    h->freq2 = freq;
    h->phase_step1 = (uint32_t)(freq * 4294967296.0f / DDS_RT_SAMPLE_RATE);
    h->phase_step2 = h->phase_step1;
}

void DDS_RT_SetAmp(DDS_RT_Handle *h, float amp_vpp) {
    h->amp1_vpp = amp_vpp;
    h->amp2_vpp = amp_vpp;
    uint32_t sc = (uint32_t)(amp_vpp * DDS_RT_Q12_SCALE * 1000.0f / 3300.0f);
    if (sc > DDS_RT_Q12_SCALE) sc = DDS_RT_Q12_SCALE;
    h->amp1_scale_q12 = (uint16_t)sc;
    h->amp2_scale_q12 = (uint16_t)sc;
}

void DDS_RT_SetPhase2(DDS_RT_Handle *h, float phase2_deg) {
    h->phase2_deg = phase2_deg;
    float pn = fmodf(phase2_deg, 360.0f);
    if (pn < 0) pn += 360.0f;
    h->phase_off2 = (uint32_t)(pn / 360.0f * 4294967296.0f);
}

void DDS_RT_SetWave2(DDS_RT_Handle *h, DDS_Wave_t wave) {
    h->wave2 = wave;
}
