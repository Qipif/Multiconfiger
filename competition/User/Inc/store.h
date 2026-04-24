#ifndef __STORE_H__
#define __STORE_H__

#include <stdint.h>
#include "mydds.h"

#define CFG_MAGIC 0xEE11CC22

// 掉电保存数据
typedef struct {
    uint32_t   magic;
    DDS_Wave_t wave;
    float      freq;
    float      amp_vpp;
    uint32_t   crc;
} CFG_Data;

// 初始化（上电调用，自动加载Flash配置）
void CFG_Init(void);

// 保存到Flash
void CFG_Save(DDS_Wave_t wave, float freq, float amp_vpp);

// 获取当前配置（RAM）
void CFG_GetActive(DDS_Wave_t *wave, float *freq, float *amp_vpp);

// 格式化Flash区
void CFG_Format(void);

#endif
