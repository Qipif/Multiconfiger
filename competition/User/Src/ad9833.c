/*
 * @description: AD9833 DDS 驱动（适配C题：信号变换与李萨如图显示装置）
 * @note: 硬件接线：PA4=FSYNC(CS), PA5=SCLK, PA7=SDATA
 *        可用硬件SPI或GPIO模拟，本驱动用GPIO模拟（简单可靠）
 */

#include "ad9833.h"
#include <math.h>

// 宏定义：GPIO操作
#define AD9833_SCLK_LOW()   HAL_GPIO_WritePin(AD9833_SCLK_Port, AD9833_SCLK_Pin, GPIO_PIN_RESET)
#define AD9833_SCLK_HIGH()  HAL_GPIO_WritePin(AD9833_SCLK_Port, AD9833_SCLK_Pin, GPIO_PIN_SET)
#define AD9833_SDATA_LOW()  HAL_GPIO_WritePin(AD9833_SDATA_Port, AD9833_SDATA_Pin, GPIO_PIN_RESET)
#define AD9833_SDATA_HIGH() HAL_GPIO_WritePin(AD9833_SDATA_Port, AD9833_SDATA_Pin, GPIO_PIN_SET)
#define AD9833_FSYNC_LOW()  HAL_GPIO_WritePin(AD9833_FSYNC_Port, AD9833_FSYNC_Pin, GPIO_PIN_RESET)
#define AD9833_FSYNC_HIGH() HAL_GPIO_WritePin(AD9833_FSYNC_Port, AD9833_FSYNC_Pin, GPIO_PIN_SET)

// AD9833 控制字
#define AD9833_CTRL_B28      (1u << 13)  // 连续写14位频率寄存器（先低后高）
#define AD9833_CTRL_HLB      (1u << 12)  // 单独写高14位（B28=0时有效）
#define AD9833_CTRL_FSEL     (1u << 11)  // 选择FREQ寄存器：0=FREQ0, 1=FREQ1
#define AD9833_CTRL_PSEL     (1u << 10)  // 选择PHASE寄存器：0=PHASE0, 1=PHASE1
#define AD9833_CTRL_PINSW    (1u << 9)   // 硬件引脚控制频率/相位选择
#define AD9833_CTRL_RESET    (1u << 8)   // 复位DDS，DAC输出=0
#define AD9833_CTRL_SLEEP1   (1u << 7)   // 关DAC输出（低功耗，但时钟不停）
#define AD9833_CTRL_SLEEP12  (1u << 6)   // 关内部MCLK（低功耗，DAC输出保持）
#define AD9833_CTRL_OPBITEN  (1u << 5)   // 输出MSB（方波）使能
#define AD9833_CTRL_SIGN_PIB (1u << 4)   // MSB输出相位：0=MSB, 1=MSB/2
#define AD9833_CTRL_DIV2     (1u << 3)   // MCLK二分频后送入相位累加器
#define AD9833_CTRL_MODE    (1u << 1)   // 波形选择：0=正弦, 1=三角

#define AD9833_FREQ_ADDR    (1u << 14)   // 写FREQ寄存器（配合B28）
#define AD9833_PHASE_ADDR   (3u << 14)   // 写PHASE寄存器

// FSYNC脉冲延时（纳秒级，GPIO翻转足够）
#define AD9833_DELAY()  do { \
        __NOP(); __NOP(); __NOP(); __NOP(); \
        __NOP(); __NOP(); __NOP(); __NOP(); \
    } while(0)

// 向AD9833写一个16位控制字（GPIO模拟SPI）
static void AD9833_WriteWord(uint16_t word)
{
    AD9833_FSYNC_LOW();
    AD9833_DELAY();

    for (int i = 15; i >= 0; i--) {
        AD9833_SCLK_LOW();
        if (word & (1u << i))
            AD9833_SDATA_HIGH();
        else
            AD9833_SDATA_LOW();
        AD9833_DELAY();
        AD9833_SCLK_HIGH();
        AD9833_DELAY();
    }

    AD9833_FSYNC_HIGH();
    AD9833_DELAY();
}

// 初始化AD9833
void AD9833_Init(AD9833_Handle *h, float mclk_mhz)
{
    // 计算频率分辨率：fout = freq_reg × MCLK / 2^28
    h->freq_step = mclk_mhz * 1000000.0f / 268435456.0f;  // = MCLK / 2^28
    h->freq      = 0;
    h->phase_reg = 0;
    h->wave      = wave_sine;

    // 硬件复位
    AD9833_WriteWord(AD9833_CTRL_RESET);
    HAL_Delay(1);

    // 解除复位，设置B28=1（连续写28位频率）
    AD9833_WriteWord(AD9833_CTRL_B28);
    HAL_Delay(1);

    // 默认输出正弦波，频率=0
    AD9833_SetFrequency(h, 0);
    AD9833_SetPhase(h, 0.0f);
    AD9833_SetWaveform(h, wave_sine);
}

// 设置输出频率（Hz）
void AD9833_SetFrequency(AD9833_Handle *h, float freq_hz)
{
    if (freq_hz < 0) freq_hz = 0;

    // 计算28位频率寄存器值
    uint32_t freq_reg = (uint32_t)(freq_hz / h->freq_step + 0.5f);
    if (freq_reg > 0x0FFFFFFF) freq_reg = 0x0FFFFFFF;

    h->freq      = freq_hz;
    h->freq_reg  = freq_reg;

    // B28=1时，先写低14位，再写高14位
    uint16_t low_14  = (uint16_t)(freq_reg & 0x3FFF) | (AD9833_FREQ_ADDR);
    uint16_t high_14 = (uint16_t)((freq_reg >> 14) & 0x3FFF) | (AD9833_FREQ_ADDR);

    AD9833_WriteWord(low_14);
    AD9833_WriteWord(high_14);
}

// 设置相位（度，0~360）
void AD9833_SetPhase(AD9833_Handle *h, float phase_deg)
{
    if (phase_deg < 0)    phase_deg = 0;
    if (phase_deg > 360) phase_deg = 360;

    // 12位相位寄存器：phase_reg = phase_deg / 360 × 4096
    uint16_t phase_reg = (uint16_t)(phase_deg / 360.0f * 4096.0f + 0.5f);
    if (phase_reg > 4095) phase_reg = 4095;

    h->phase_reg = phase_reg;

    uint16_t cmd = (phase_reg & 0x0FFF) | AD9833_PHASE_ADDR;
    AD9833_WriteWord(cmd);
}

// 设置输出波形
void AD9833_SetWaveform(AD9833_Handle *h, WaveType wave)
{
    h->wave = wave;

    uint16_t ctrl = AD9833_CTRL_B28;  // 基础控制字（B28=1，不复位）

    switch (wave) {
        case wave_sine:
            // MODE=0, OPBITEN=0 → 正弦波
            break;
        case wave_triangle:
            // MODE=1 → 三角波
            ctrl |= AD9833_CTRL_MODE;
            break;
        case wave_square:
            // OPBITEN=1, MODE=0 → MSB（方波）
            ctrl |= AD9833_CTRL_OPBITEN;
            break;
    }

    AD9833_WriteWord(ctrl);
}
