#ifndef __MYDRAW_H__
#define __MYDRAW_H__

#include "main.h"
#include "oled.h"

// 初始化显示
void MYDRAW_Init(void);

// 李萨如图：ch_x→X轴, ch_y→Y轴, len=采样点数
// 12位ADC数据，2048为中点
void MYDRAW_DrawLissajous(const uint16_t *ch_x, const uint16_t *ch_y, uint16_t len);

#endif
