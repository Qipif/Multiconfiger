#include "encoder.h"

static TIM_HandleTypeDef *enc_tim = NULL;
static int32_t encoder_count = 0;
static int16_t last_cnt = 0;
static int16_t remain = 0;    // 累积余数

void ENCODER_Init(TIM_HandleTypeDef *htim) {
    enc_tim = htim;
    HAL_TIM_Encoder_Start(enc_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(enc_tim, 0);
    last_cnt = 0;
    encoder_count = 0;
    remain = 0;
}

int16_t ENCODER_GetDelta(void) {
    if (!enc_tim) return 0;
    int16_t cur_cnt = (int16_t)__HAL_TIM_GET_COUNTER(enc_tim);
    int16_t diff = cur_cnt - last_cnt;
    last_cnt = cur_cnt;

    remain += diff;
    int16_t delta = remain / 4;   // 机械步数（4倍频）
    remain %= 4;

    encoder_count += delta;
    return delta;
}

int32_t ENCODER_GetCount(void) {
    return encoder_count;
}

void ENCODER_SetCount(int32_t value) {
    encoder_count = value;
}
