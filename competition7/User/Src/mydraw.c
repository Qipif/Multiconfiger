#include "mydraw.h"
#include <string.h>

void MYDRAW_Init(void)
{
}

// 李萨如图：全屏128×64显示
// 优化：逐点写GRAM，最后一次性ShowFrame
// 高频修复：只画128个点（降采样），避免256个点叠成糊
void MYDRAW_DrawLissajous(const uint16_t *ch_x, const uint16_t *ch_y, uint16_t len)
{
    OLED_NewFrame();

    // 降采样到128点：均匀抽取
    uint16_t step = (len > 128) ? (len / 128) : 1;

    for (uint16_t i = 0; i < 128 && i * step < len; i++) {
        uint16_t idx = i * step;
        // 12位ADC，2048为中点
        int16_t vx = (int16_t)ch_x[idx] - 2048;
        int16_t vy = (int16_t)ch_y[idx] - 2048;
        // 映射到128×64屏幕，中心对齐
        int16_t x = 64 + ((int32_t)vx * 60 / 2048);
        int16_t y = 32 + ((int32_t)vy * 28 / 2048);
        if (x < 2) x = 2;
        if (x > 125) x = 125;
        if (y < 2) y = 2;
        if (y > 61) y = 61;
        OLED_SetPixel(x, y, OLED_COLOR_NORMAL);
        // 画粗一点：上下左右各1像素
        OLED_SetPixel(x+1, y, OLED_COLOR_NORMAL);
        OLED_SetPixel(x-1, y, OLED_COLOR_NORMAL);
        OLED_SetPixel(x, y+1, OLED_COLOR_NORMAL);
        OLED_SetPixel(x, y-1, OLED_COLOR_NORMAL);
    }

    OLED_ShowFrame();
}
