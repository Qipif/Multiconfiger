#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "tim.h"

void ENCODER_Init(TIM_HandleTypeDef *htim);
int16_t ENCODER_GetDelta(void);
int32_t ENCODER_GetCount(void);
void ENCODER_SetCount(int32_t value);

#endif
