#include "mydraw.h"
#include <stdio.h>
#include <string.h>

static DrawMode current_mode = DISP_MODE_WAVE_IN;

void MYDRAW_Init(void) {
    OLED_Clear();
}

void MYDRAW_SetMode(DrawMode mode) {
    current_mode = mode;
    OLED_Clear();
}

DrawMode MYDRAW_GetMode(void) {
    return current_mode;
}

void MYDRAW_NextMode(void) {
    current_mode = (current_mode + 1) % 4;  // 0,1,2,3 循环
    OLED_Clear();
}

void MYDRAW_DrawSingleWave(const uint16_t *ch, uint16_t len, uint8_t channel) {
    OLED_NewFrame();
    for (uint16_t x = 0; x < 128; x++) {
        uint32_t idx = (uint32_t)x * len / 128;
        if (idx >= len) idx = len - 1;
        uint16_t val = ch[idx];
        uint8_t y = (uint8_t)(val * 63 / 4095);
        y = 63 - y;
        OLED_SetPixel(x, y, OLED_COLOR_NORMAL);
    }
//    char label[8];
//    sprintf(label, "CH%d", channel);
//    OLED_ShowString(1, 1, label);
    OLED_ShowFrame();
}

void MYDRAW_DrawLissajous(const uint16_t *ch0, const uint16_t *ch1, uint16_t len) {
    #define LISSA_X_MIN 32
    #define LISSA_X_MAX 95
    #define LISSA_Y_MIN 0
    #define LISSA_Y_MAX 63

    OLED_NewFrame();
    for (uint16_t i = 0; i < len; i++) {
        int16_t val0 = (int16_t)ch0[i] - 2048;
        int16_t val1 = (int16_t)ch1[i] - 2048;
        int16_t x = LISSA_X_MIN + 32 + (val0 * 32 / 2048);
        int16_t y = LISSA_Y_MIN + 32 + (val1 * 32 / 2048);
        if (x < LISSA_X_MIN) x = LISSA_X_MIN;
        if (x > LISSA_X_MAX) x = LISSA_X_MAX;
        if (y < LISSA_Y_MIN) y = LISSA_Y_MIN;
        if (y > LISSA_Y_MAX) y = LISSA_Y_MAX;
        OLED_SetPixel(x, y, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}

void MYDRAW_DrawFFTInfoEx(const FFT_Result *r0,
                          const FFT_Result *r1,
                          float phase_diff_deg,
                          int gain)
{
    char buf[17];

    // 行1：增益
    sprintf(buf, "Gain: x%d", gain);
    OLED_ShowString(1, 1, buf);

    // 行2：输入
    sprintf(buf, "IN : %s %4.0fHz",
            FFT_WaveStr(r0->wave_type),
            r0->frequency);
    OLED_ShowString(2, 1, buf);

    // 行3：输出
    sprintf(buf, "OUT: %s %4.0fHz",
            FFT_WaveStr(r1->wave_type),
            r1->frequency);
    OLED_ShowString(3, 1, buf);

    // 行4：相位
    sprintf(buf, "PH : %+5.1f", phase_diff_deg);
    OLED_ShowString(4, 1, buf);
}

/**
 * @brief 绘制频谱柱状图
 * @param mag     FFT幅度谱（fft_len/2项，电压V）
 * @param fft_len FFT点数(256或1024)
 */
void MYDRAW_DrawSpectrum(const float *mag, uint16_t fft_len) {
    if (!mag || fft_len < 128) return;

    #define SPEC_TOP_Y    10
    #define SPEC_HEIGHT   48
    #define SPEC_BOTTOM_Y (SPEC_TOP_Y + SPEC_HEIGHT - 1)

    uint16_t n_bins = fft_len / 2;
    uint16_t bars = 64;            // 64根柱=128像素宽
    uint16_t bins_per_bar = n_bins / bars;  // 256pt→2, 1024pt→8

    OLED_NewFrame();

    for (uint16_t x = 0; x < bars; x++) {
        uint16_t bin = x * bins_per_bar;
        if (bin >= n_bins) break;

        float mag_val = mag[bin];
        int height = (int)(mag_val * (float)SPEC_HEIGHT / 2.0f);
        if (height > SPEC_HEIGHT) height = SPEC_HEIGHT;
        if (height < 0) height = 0;

        int col_l = x * 2;
        int col_r = x * 2 + 1;
        for (int y = 0; y < height; y++) {
            int row = SPEC_BOTTOM_Y - y;
            OLED_SetPixel(col_l, row, OLED_COLOR_NORMAL);
            OLED_SetPixel(col_r, row, OLED_COLOR_NORMAL);
        }
    }

    OLED_ShowFrame();
}
