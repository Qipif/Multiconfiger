#ifndef __STORE_H__
#define __STORE_H__

#include <stdint.h>

#define CFG_MAGIC 0xEE11CC22

// 掉电保存数据（只存校准值和周期值）
typedef struct {
    uint32_t   magic;
    float      phase_offset;   // 相位校准偏移（模式0校准后存储）
    float      auto_time;      // 自动循环周期（模式3确认后存储）
    uint32_t   crc;
} CFG_Data;

// 初始化（上电调用，自动加载Flash配置）
void CFG_Init(void);

// 保存到Flash
void CFG_Save(float phase_offset, float auto_time);

// 获取当前配置（RAM）
void CFG_GetActive(float *phase_offset, float *auto_time);

#endif
