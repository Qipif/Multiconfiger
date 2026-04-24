#ifndef __MYDRAW_H__
#define __MYDRAW_H__

#include "main.h"
#include "oled.h"
#include "myfft.h"

typedef enum {
    DISP_MODE_WAVE_IN = 0,
    DISP_MODE_WAVE_OUT,
    DISP_MODE_FFT_INFO,
    DISP_MODE_LISSA       // 预留
} DrawMode;

void MYDRAW_Init(void);
void MYDRAW_SetMode(DrawMode mode);
DrawMode MYDRAW_GetMode(void);
void MYDRAW_NextMode(void);

void MYDRAW_DrawSingleWave(const uint16_t *ch, uint16_t len, uint8_t channel);
void MYDRAW_DrawLissajous(const uint16_t *ch0, const uint16_t *ch1, uint16_t len);
void MYDRAW_DrawFFTInfoEx(const FFT_Result *r0,
                          const FFT_Result *r1,
                          float phase_diff_deg,
                          int gain);
void MYDRAW_DrawSpectrum(const FFT_Result *result, uint16_t fft_len);

#endif
