#include "mydraw.h"
#include <stdio.h>
#include <string.h>

static DrawMode current_mode = DISP_MODE_WAVE;

void MYDRAW_Init(void) {
    // OLED 初始化已在外部完成，此处可留空或添加清屏
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
    current_mode = (current_mode + 1) % 4;
    OLED_Clear();
}

void MYDRAW_DrawWaveOverlay(const uint16_t *ch0, const uint16_t *ch1, uint16_t len) {
    OLED_NewFrame();
    for (uint16_t x = 0; x < len; x++) {
        uint32_t sum = ch0[x] + ch1[x];
        uint16_t combined = sum / 2;
        uint8_t y = combined * 63 / 4095;   // 0~63
        y = 63 - y;
        OLED_SetPixel(x, y, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}

void MYDRAW_DrawLissajous(const uint16_t *ch0, const uint16_t *ch1, uint16_t len) {
    #define LISSA_X_MIN 32
    #define LISSA_X_MAX 95
    #define LISSA_Y_MIN 0
    #define LISSA_Y_MAX 63

    OLED_NewFrame();
    int16_t last_x = -1, last_y = -1;
    for (uint16_t i = 0; i < len; i++) {
        // 去直流并映射到 -2048..+2047 范围，再缩放到 ±32
        int16_t val0 = (int16_t)ch0[i] - 2048;
        int16_t val1 = (int16_t)ch1[i] - 2048;
        int16_t x = LISSA_X_MIN + 32 + (val0 * 32 / 2048);
        int16_t y = LISSA_Y_MIN + 32 + (val1 * 32 / 2048);
        if (x < LISSA_X_MIN) x = LISSA_X_MIN;
        if (x > LISSA_X_MAX) x = LISSA_X_MAX;
        if (y < LISSA_Y_MIN) y = LISSA_Y_MIN;
        if (y > LISSA_Y_MAX) y = LISSA_Y_MAX;
        OLED_SetPixel(x, y, OLED_COLOR_NORMAL);
        // 可选的连线功能：若有 OLED_DrawLine 可启用
        last_x = x;
        last_y = y;
    }
    OLED_ShowFrame();
}

void MYDRAW_DrawFFTInfo(MYFFT_Handle *fft, const uint16_t *ch0, const uint16_t *ch1,
                        uint16_t len, uint16_t fft_len, MYFFT_Result *result) {
    // 准备 FFT 输入缓冲
    uint16_t *fft_in = (uint16_t*)malloc(fft_len * sizeof(uint16_t));
    if (!fft_in) return;

    // 分析通道0
    for (uint16_t i = 0; i < len; i++) fft_in[i] = ch0[i];
    for (uint16_t i = len; i < fft_len; i++) fft_in[i] = 2048;
    MYFFT_Process(fft, fft_in, result);
    float freq0 = result->frequency;
    float amp0  = result->amplitude;
    float phase0_deg = result->phase * 180.0f / 3.1415926f;

    // 分析通道1
    for (uint16_t i = 0; i < len; i++) fft_in[i] = ch1[i];
    for (uint16_t i = len; i < fft_len; i++) fft_in[i] = 2048;
    MYFFT_Process(fft, fft_in, result);
    float freq1 = result->frequency;
    float amp1  = result->amplitude;
    float phase1_deg = result->phase * 180.0f / 3.1415926f;

    free(fft_in);

    float phase_diff = phase0_deg - phase1_deg;
    while (phase_diff > 180.0f) phase_diff -= 360.0f;
    while (phase_diff < -180.0f) phase_diff += 360.0f;

    OLED_NewFrame();
    char buf[17];
    sprintf(buf, "F0:%4.0f A0:%4.2f", freq0, amp0);
    OLED_ShowString(0, 0, buf);
    sprintf(buf, "F1:%4.0f A1:%4.2f", freq1, amp1);
    OLED_ShowString(0, 1, buf);
    sprintf(buf, "P0:%+5.0f P1:%+5.0f", phase0_deg, phase1_deg);
    OLED_ShowString(0, 2, buf);
    sprintf(buf, "Diff:%+6.1f", phase_diff);
    OLED_ShowString(0, 3, buf);
    OLED_ShowFrame();
}

void MYDRAW_DrawSpectrum(const MYFFT_Result *result, uint16_t fft_len) {
    #define SPEC_TOP_Y      10
    #define SPEC_HEIGHT     48
    #define SPEC_BOTTOM_Y   (SPEC_TOP_Y + SPEC_HEIGHT - 1)

    OLED_NewFrame();
    char buf[17];
    sprintf(buf, "F:%5.0fHz %4.2fV", result->frequency, result->amplitude);
    OLED_ShowString(0, 0, buf);

    for (int x = 0; x < 64; x++) {
        int bin = x * 4;
        if (bin >= fft_len/2) break;
        float mag = result->mag[bin];
        int height = (int)(mag * SPEC_HEIGHT / 2.0f);
        if (height > SPEC_HEIGHT) height = SPEC_HEIGHT;
        if (height < 0) height = 0;
        int col_left  = x * 2;
        int col_right = x * 2 + 1;
        for (int y = 0; y < height; y++) {
            int row = SPEC_BOTTOM_Y - y;
            OLED_SetPixel(col_left,  row, OLED_COLOR_NORMAL);
            OLED_SetPixel(col_right, row, OLED_COLOR_NORMAL);
        }
    }
    OLED_ShowFrame();
}
