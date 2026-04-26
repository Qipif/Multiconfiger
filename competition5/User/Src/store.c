#include "main.h"
#include "store.h"
#include <string.h>

// STM32H7 Bank2 Sector7 最后128KB用于配置
#define CFG_FLASH_SECTOR  FLASH_SECTOR_7
#define CFG_FLASH_ADDR    0x081E0000
#define CFG_FLASH_BANK    FLASH_BANK_2

static CFG_Data active_cfg;

// 32字节对齐的写入缓冲（避免读越界）
static uint8_t _flash_buf[32];

// CRC32
static uint32_t _crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

// 写Flash
static uint8_t _write_flash(const CFG_Data *d) {
    HAL_FLASH_Unlock();

    // 擦除扇区
    FLASH_EraseInitTypeDef erase;
    uint32_t err;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = CFG_FLASH_SECTOR;
    erase.Banks = CFG_FLASH_BANK;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return 1;
    }

    // 拷贝到对齐缓冲，零填充
    memset(_flash_buf, 0, sizeof(_flash_buf));
    memcpy(_flash_buf, d, sizeof(CFG_Data));

    // 32字节一行（8个uint32_t = 256bit FlashWord）
    uint32_t *src = (uint32_t*)_flash_buf;
    uint32_t n = (sizeof(CFG_Data) + 31) / 32;

    for (uint32_t i = 0; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              CFG_FLASH_ADDR + i * 32,
                              (uint32_t)&src[i * 8]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 2;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

void CFG_Init(void) {
    CFG_Data *flash = (CFG_Data*)CFG_FLASH_ADDR;
    if (flash->magic == CFG_MAGIC) {
        uint32_t crc = _crc32((const uint8_t*)flash, sizeof(CFG_Data) - 4);
        if (crc == flash->crc) {
            active_cfg = *flash;
            return;
        }
    }
    // Flash无效，用默认值
    active_cfg.magic = CFG_MAGIC;
    active_cfg.phase_offset = 0.0f;
    active_cfg.auto_time = 5.0f;
}

void CFG_Save(float phase_offset, float auto_time) {
    active_cfg.magic = CFG_MAGIC;
    active_cfg.phase_offset = phase_offset;
    active_cfg.auto_time = auto_time;
    active_cfg.crc = _crc32((const uint8_t*)&active_cfg, sizeof(CFG_Data) - 4);
    _write_flash(&active_cfg);
}

void CFG_GetActive(float *phase_offset, float *auto_time) {
    if (phase_offset) *phase_offset = active_cfg.phase_offset;
    if (auto_time)    *auto_time = active_cfg.auto_time;
}
