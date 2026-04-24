# Multiconfiger 用户手册

> 硬件平台：STM32H743VIT6 | 12位ADC/DAC | SSD1306 OLED 128×64 | 旋转编码器(TIM1+PA0)
>
> 项目总览：
> - **10_multiconfiger10**：双算法DDS + 多模式FFT + Flash预设 + OLED显示 + 编码器控制
> - **11_multiconfiger11**：CPLL(过零硬同步DDS+内部比较器) + DPLL
> - **12_multiconfiger12**：APLL(数字延迟线锁相)，**完美验证**

---

## 目录

1. [mydds — DDS波形发生器](#1-mydds--dds波形发生器)
2. [myfft — FFT频谱分析](#2-myfft--fft频谱分析)
3. [mydraw — OLED显示绘制](#3-mydraw--oled显示绘制)
4. [encoder — 旋转编码器](#4-encoder--旋转编码器)
5. [store — Flash预设存储](#5-store--flash预设存储)
6. [完整示例：mainoop.c 搭建流程](#6-完整示例mainoopc-搭建流程)
7. [硬件资源映射表](#7-硬件资源映射表)
8. [锁相方案：CPLL与APLL](#8-锁相方案cpll与apll)
9. [常见问题](#9-常见问题)

---

## 1. mydds — DDS波形发生器

### 1.1 功能概述

用TIM+DAC输出任意频率/幅度/相位的正弦、方波、三角波。支持两种算法：

| 算法 | 宏 | 特点 | 适用频率 |
|------|-----|------|---------|
| 快速查表 | `DDS_ALGO_FAST` | 256点查表+插值，响应快 | 10kHz以下 |
| 高精度累加 | `DDS_ALGO_PRECISE` | 32位相位累加，频率精确 | 100kHz |

### 1.2 初始化

```c
#include "mydds.h"

static DDS_Handle dds1;

// 方式一：简单初始化（默认FAST算法+256点）
DDS_Init(&dds1, &htim4, &hdac1, DAC_CHANNEL_1);

// 方式二：带配置初始化（推荐）
DDS_Config dcfg = {
    .algo            = DDS_ALGO_FAST,      // 算法
    .wave            = DDS_SINE,            // 初始波形
    .table_size      = 256,                 // 查表点数 (128/256/512)
    .target_rate     = 10000000UL,          // DAC更新率 (5~20MHz)
    .auto_switch_freq = 0,                  // 自动切换阈值 (0=关闭)
    .enable_interp   = 1                    // 线性插值 (推荐开启)
};
DDS_InitWithConfig(&dds1, &dcfg, &htim4, &hdac1, DAC_CHANNEL_1);
```

**参数说明：**
- `table_size`：查表点数，越大波形越精细，但计算略慢。256是平衡选择
- `target_rate`：DAC定时器目标更新率，实际值受TIM分频限制。10MHz适合10kHz以下
- `auto_switch_freq`：设0关闭；设某频率值时，低于该频率自动切PRECISE算法
- `enable_interp`：1=开启线性插值，减少查表台阶噪声

### 1.3 设置波形

```c
// 全参数设置（波形+频率+幅度+相位，一次性改）
DDS_SetWave(&dds1, DDS_SINE, 1000.0f, 2.0f, 0.0f);
//                  波形类型   频率Hz  Vp-p   相位度

// 独立调参（更快，不改其他参数）
DDS_SetFreq(&dds1, 5000.0f);       // 只改频率
DDS_SetAmp(&dds1, 1.5f);           // 只改幅度
DDS_SetPhase(&dds1, 90.0f);        // 只改相位

// 通用接口
DDS_SetParam(&dds1, DDS_PARAM_FREQ, 2000.0f);
DDS_SetParam(&dds1, DDS_PARAM_AMP,  3.0f);
DDS_SetParam(&dds1, DDS_PARAM_PHASE, 45.0f);
```

**波形类型枚举：**
| 值 | 含义 |
|----|------|
| `DDS_SINE` | 正弦波 |
| `DDS_TRIANGLE` | 三角波 |
| `DDS_SQUARE` | 方波 |

**参数范围：**
- 频率：10Hz ~ 200kHz（FAST模式下建议10kHz以下）
- 幅度：0.0 ~ 3.3V（Vp-p，受DAC参考电压限制）
- 相位：0.0 ~ 360.0°

### 1.4 运行时切换算法

```c
DDS_SetAlgo(&dds1, DDS_ALGO_PRECISE);  // 切到高精度模式
```

### 1.5 启停控制

```c
DDS_Start(&dds1);   // 开始输出
DDS_Stop(&dds1);    // 停止输出
```

> 注意：`DDS_SetWave`内部会自动Start，一般不需要手动调。

### 1.6 工具函数

```c
const char *s = DDS_WaveStr(DDS_SINE);   // 返回 "SIN"
const char *a = DDS_AlgoStr(DDS_ALGO_FAST);  // 返回算法名称
```

### 1.7 API快速索引

| 函数 | 功能 |
|------|------|
| `DDS_Init()` | 简单初始化 |
| `DDS_InitWithConfig()` | 带配置初始化 |
| `DDS_SetWave()` | 设置波形+频率+幅度+相位 |
| `DDS_SetFreq()` | 只改频率 |
| `DDS_SetAmp()` | 只改幅度 |
| `DDS_SetPhase()` | 只改相位 |
| `DDS_SetParam()` | 通用参数设置 |
| `DDS_SetAlgo()` | 切换算法 |
| `DDS_Start()` | 开始输出 |
| `DDS_Stop()` | 停止输出 |
| `DDS_WaveStr()` | 波形名称字符串 |
| `DDS_AlgoStr()` | 算法名称字符串 |

---

## 2. myfft — FFT频谱分析

### 2.1 功能概述

对ADC采集的数据做FFT分析，输出：频率、幅值、相位、波形识别、双信号分离。

**双模式：**

| 模式 | 宏 | FFT点数 | 窗函数 | 频率分辨率(64kHz采样) | 适用场景 |
|------|-----|---------|--------|---------------------|---------|
| 快速 | `FFT_MODE_FAST` | 256 | 矩形窗 | 250Hz | 快速测频/波形识别 |
| 精准 | `FFT_MODE_PRECISE` | 1024 | 汉宁窗 | 62.5Hz | 精确幅值/相位 |

### 2.2 初始化

```c
#include "myfft.h"

static FFT_Handle *fft_handle;

// 初始化（静态内存，不用malloc）
// 参数：采样率, ADC参考电压, ADC位数
fft_handle = MYFFT_Init(64000.0f, 3.3f, 12);

// 选择模式
MYFFT_SetMode(fft_handle, FFT_MODE_PRECISE);
```

**参数说明：**
- `sample_rate`：ADC实际采样率，影响频率计算。必须和TIM触发频率一致
- `adc_vref`：ADC参考电压，H743默认3.3V
- `adc_bits`：ADC分辨率，H743默认12位

### 2.3 运行时配置

```c
MYFFT_SetMode(fft_handle, FFT_MODE_FAST);       // 切快速模式
MYFFT_SetMode(fft_handle, FFT_MODE_PRECISE);     // 切精准模式
MYFFT_SetSampleRate(fft_handle, 128000.0f);       // 改采样率（只改数值，不改变TIM）
MYFFT_SetWindow(fft_handle, FFT_WIN_HANNING);     // 改窗函数（一般不需要，模式自带）
```

> **重要**：`MYFFT_SetSampleRate`只修改FFT内部的采样率数值，不会改变硬件定时器。改采样率后需要自己用`TIM_Set_Frequency()`改TIM。

### 2.4 单信号处理

```c
FFT_Result result;
MYFFT_Process(fft_handle, adc_buf, &result);

// 读取结果
float freq = result.frequency;        // 基频 (Hz)
float amp  = result.amplitude;        // 幅值 (V, 有效值×2≈Vpp)
float pha  = result.phase;            // 相位 (rad)
FFT_Wave_t wave = result.wave_type;   // 波形类型
float ms   = result.process_ms;       // 处理耗时
```

**波形识别结果：**
| 值 | 含义 |
|----|------|
| `FFT_SINE` | 正弦波 |
| `FFT_SQUARE` | 方波 |
| `FFT_TRIANGLE` | 三角波 |
| `FFT_UNKNOWN` | 无法识别 |

### 2.5 双信号分离

```c
FFT_Result r_low, r_high;
MYFFT_ProcessDual(fft_handle, adc_buf, &r_low, &r_high);

// r_low:  低频信号 (频率较小的那个)
// r_high: 高频信号 (频率较大的那个)
```

**双信号分离原理：**
1. 先做一次FFT找到主峰
2. 备份mag数组，屏蔽主峰附近5个bin
3. 重新找第二峰
4. 按频率排序输出

### 2.6 ADC采集配置

FFT需要配合ADC+DMA循环采集。以下是完整配置：

```c
// 1. 定义缓冲（双缓冲，1024×2）
#define FFT_ADC_BUF_LEN (1024 * 2)
static uint16_t fft_adc_buf[FFT_ADC_BUF_LEN];
static volatile uint8_t fft_ready = 0;
static volatile uint8_t fft_half  = 0;

// 2. 初始化：校准+启动DMA+启动TIM
HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
HAL_ADC_Start_DMA(&hadc2, (uint32_t*)fft_adc_buf, FFT_ADC_BUF_LEN);
HAL_TIM_Base_Start(&htim3);

// 3. 改采样率（可选，默认64kHz）
// TIM_Set_Frequency(&htim3, 64000);

// 4. DMA回调
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) { fft_half = 0; fft_ready = 1; }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC2) { fft_half = 1; fft_ready = 1; }
}

// 5. 主循环处理
if (fft_ready) {
    fft_ready = 0;
    uint16_t *data = (fft_half == 0)
        ? &fft_adc_buf[0]
        : &fft_adc_buf[1024];
    MYFFT_Process(fft_handle, data, &fft_result);
}
```

### 2.7 频率分辨率说明

FFT的频率分辨率 = 采样率 / 点数，只能报告bin中心频率。

| 采样率 | 点数 | 分辨率 | 1000Hz真实频率 → FFT报告 |
|--------|------|--------|------------------------|
| 64kHz | 256 | 250Hz | 1000Hz（bin 4，精确） |
| 64kHz | 1024 | 62.5Hz | 1000Hz（bin 16，精确） |
| 64kHz | 1024 | 62.5Hz | 1040Hz → 1062Hz（最近bin） |

当信号频率不在bin中心时，FFT报告值会"吸"到最近的62.5Hz整数倍，这是正常现象。

### 2.8 Goertzel算法（单频快速测量）

Goertzel算法是FFT的O(N)替代方案，专门用于单频信号检测。比FFT快4~8倍，适合DDS输出验证。

**与FFT对比：**

| 特性 | FFT | Goertzel |
|------|-----|----------|
| 复杂度 | O(N log N) | O(N) |
| 适用场景 | 宽带频谱 | 单频检测 |
| 精度 | bin中心频率 | 精确目标频率 |
| 双信号 | 支持 | ❌ 只支持单频 |

**初始化：**

```c
// Goertzel使用前需要设置目标频率bin
// bin = freq * 256 / sample_rate
// 例: 1kHz @ 32.7kHz → bin ≈ 8
MYFFT_Goertzel_SetFreqBin(8);
```

**处理相位差：**

```c
static float phase_diff_f = 0.0f;
FFT_Result result;

// 双通道交错数据: [ch1,ch2,ch1,ch2,...]
MYFFT_Goertzel_ProcessPhase(fft_handle, dual_adc_buf,
                            0.5f,        // 相位偏置补偿
                            &phase_diff_f, // 滤波后的相位差
                            &result);

// result.amplitude: 幅值
// result.frequency: 频率
// result.phase_diff: 滤波+补偿后的相位差
```

**调频时更新bin：**

```c
void on_freq_changed(float new_freq) {
    uint16_t bin = (uint16_t)(new_freq * 256 / sample_rate);
    MYFFT_Goertzel_SetFreqBin(bin);
}
```

### 2.9 工具函数

```c
const char *s = FFT_WaveStr(FFT_SINE);     // 返回 "SIN"
const char *m = FFT_ModeStr(FFT_MODE_FAST); // 返回模式名称
```

### 2.10 API快速索引

| 函数 | 功能 |
|------|------|
| `MYFFT_Init()` | 初始化 |
| `MYFFT_SetMode()` | 切换FAST/PRECISE模式 |
| `MYFFT_Process()` | 单信号FFT分析 |
| `MYFFT_ProcessDual()` | 双信号分离 |
| `MYFFT_ProcessPhase()` | FFT双通道相位差 |
| `MYFFT_Goertzel_ProcessPhase()` | Goertzel相位差（更快） |
| `MYFFT_Goertzel_SetFreqBin()` | 设置Goertzel目标频率 |
| `MYFFT_SetPhaseOffset()` | 相位偏置校准 |
| `MYFFT_SetSampleRate()` | 修改采样率数值（不改硬件） |
| `MYFFT_SetWindow()` | 修改窗函数 |
| `FFT_WaveStr()` | 波形名称字符串 |
| `FFT_ModeStr()` | 模式名称字符串 |

---

## 3. mydraw — OLED显示绘制

### 3.1 功能概述

在SSD1306 OLED（128×64）上画波形、李萨如图、FFT信息、频谱。使用帧缓冲，支持像素级操作。

### 3.2 初始化

```c
#include "mydraw.h"

MYDRAW_Init();   // 清屏
```

> `MYDRAW_Init`内部调`OLED_Clear()`，OLED本身的`OLED_Init()`需要在之前调用。

### 3.3 显示模式

```c
typedef enum {
    DISP_MODE_WAVE_IN  = 0,  // 输入波形
    DISP_MODE_WAVE_OUT,       // 输出波形
    DISP_MODE_FFT_INFO,       // FFT信息
    DISP_MODE_LISSA           // 李萨如
} DrawMode;

// 设置/切换模式
MYDRAW_SetMode(DISP_MODE_WAVE_IN);
MYDRAW_NextMode();    // 循环切换 0→1→2→3→0
DrawMode m = MYDRAW_GetMode();
```

### 3.4 画单通道波形

```c
// data: ADC原始数据 (uint16_t, 0~4095)
// len:  数据长度
// ch:   通道号（0或1，用于标签显示）

uint16_t adc_data[128];
MYDRAW_DrawSingleWave(adc_data, 128, 0);
```

**实现细节：**
- 128像素横轴映射到len个采样点
- 12位ADC值线性映射到64像素纵轴（0=顶部, 63=底部）
- 使用帧缓冲：`OLED_NewFrame()` → 逐像素 `OLED_SetPixel()` → `OLED_ShowFrame()`

### 3.5 画李萨如图

```c
// ch0, ch1: 两路ADC数据
// len: 数据长度（两路相同）

uint16_t adc_ch0[128], adc_ch1[128];
MYDRAW_DrawLissajous(adc_ch0, adc_ch1, 128);
```

**实现细节：**
- 以(2048, 2048)为零点，取±2048范围映射到64×64像素区域
- 适合观察两路同频信号的相位差（椭圆→直线→圆）

### 3.6 显示FFT信息（双通道对比）

```c
FFT_Result r_input, r_output;
float phase_diff = 30.5f;
int gain = 2;

MYDRAW_DrawFFTInfoEx(&r_input, &r_output, phase_diff, gain);
```

**OLED显示效果：**
```
行1: Gain: x2
行2: IN : SIN 1000Hz
行3: OUT: SIN 1000Hz
行4: PH : +30.5
```

### 3.7 显示频谱（文字版）

```c
FFT_Result result;
MYDRAW_DrawSpectrum(&result, 1024);
```

**OLED显示效果：**
```
行1: F: 1000Hz
行2: A:1.00V
行3: SIN
```

### 3.8 API快速索引

| 函数 | 功能 |
|------|------|
| `MYDRAW_Init()` | 清屏初始化 |
| `MYDRAW_SetMode()` | 设置显示模式 |
| `MYDRAW_NextMode()` | 循环切换模式 |
| `MYDRAW_GetMode()` | 获取当前模式 |
| `MYDRAW_DrawSingleWave()` | 画单通道波形 |
| `MYDRAW_DrawLissajous()` | 画李萨如图 |
| `MYDRAW_DrawFFTInfoEx()` | 显示FFT双通道信息 |
| `MYDRAW_DrawSpectrum()` | 显示频谱文字 |

---

## 4. encoder — 旋转编码器

### 4.1 功能概述

旋转编码器通过TIM1计数器检测旋转方向和格数，PA0按键检测短按/长按/双击。

### 4.2 初始化

```c
#include "encoder.h"

static ENC_Handle enc1;

// htim1: TIM1（编码器模式，PE9/PE11）
// KEY_GPIO_Port, KEY_Pin: 按键GPIO（PA0）
ENC_Init(&enc1, &htim1, KEY_GPIO_Port, KEY_Pin);
```

**默认参数（初始化后）：**
| 参数 | 默认值 |
|------|--------|
| 控制目标 | `ENC_CTRL_FREQ` |
| 频率步进 | 10/100/1000 Hz（慢/中/快） |
| 幅度步进 | 0.1/0.5/1.0 V |
| 相位步进 | 1/10/30 ° |
| 频率范围 | 10Hz ~ 200kHz |
| 幅度范围 | 0.1V ~ 3.3V |
| 相位范围 | 0° ~ 360° |

### 4.3 主循环更新

```c
// 每帧调用，返回按键事件
ENC_Event_t evt = ENC_Update(&enc1);

// 处理按键事件
if (evt == ENC_EVT_CLICK) {
    ENC_NextTarget(&enc1);        // 短按：切换控制目标
}
if (evt == ENC_EVT_LONG_PRESS) {
    // 长按：进预设模式（自定义逻辑）
}
if (evt == ENC_EVT_DOUBLE_CLICK) {
    // 双击：退出（自定义逻辑）
}
```

**按键事件：**
| 事件 | 触发条件 |
|------|---------|
| `ENC_EVT_NONE` | 无按键 |
| `ENC_EVT_CLICK` | 短按释放（<1s） |
| `ENC_EVT_LONG_PRESS` | 持续按下>1s |
| `ENC_EVT_DOUBLE_CLICK` | 两次短按间隔<300ms |

### 4.4 旋转同步到DDS

```c
// 只在旋转时同步（推荐，避免卡屏）
if (enc1.rotated) {
    enc1.rotated = 0;
    ENC_ApplyToDDS(&enc1, &dds1);
}
```

> **重要**：`ENC_ApplyToDDS`会根据当前target调用`DDS_SetWave`/`DDS_SetFreq`等。如果每帧都调，切到WAVE模式时每帧重建波形表会严重卡屏。务必用`rotated`标志过滤。

### 4.5 控制目标

```c
// 切换控制目标（循环：FREQ→AMP→PHASE→WAVE→PRESET→FFT_MODE→FREQ）
ENC_NextTarget(&enc1);

// 直接设置目标
ENC_SetTarget(&enc1, ENC_CTRL_WAVE);

// 查询当前目标
ENC_CtrlTarget_t t = ENC_GetTarget(&enc1);
```

**控制目标列表：**
| 编号 | 枚举 | 旋转效果 |
|------|------|---------|
| 1 | `ENC_CTRL_FREQ` | 频率 ±10Hz（慢速档） |
| 2 | `ENC_CTRL_AMP` | 幅度 ±0.1V |
| 3 | `ENC_CTRL_PHASE` | 相位 ±1° |
| 4 | `ENC_CTRL_WAVE` | 波形循环 SIN→SQR→TRI |
| 5 | `ENC_CTRL_PRESET` | 预设切换 |
| 6 | `ENC_CTRL_FFT_MODE` | FAST↔PRECISE |

### 4.6 旋转速度档位

```c
uint8_t gear = ENC_GetGear(&enc1);  // 0=慢 1=中 2=快
```

档位由短时间内旋转格数决定：
- 慢速（gear=0）：使用`step_slow`步进
- 中速（gear=1）：使用`step_mid`步进
- 快速（gear=2）：使用`step_fast`步进

### 4.7 手动读取编码器值

```c
float freq  = enc1.cur_freq;         // 当前频率
float amp   = enc1.cur_amp;          // 当前幅度
float phase = enc1.cur_phase;        // 当前相位
DDS_Wave_t wave = enc1.cur_wave;     // 当前波形
FFT_Mode_t mode = enc1.cur_fft_mode; // 当前FFT模式
```

### 4.8 API快速索引

| 函数 | 功能 |
|------|------|
| `ENC_Init()` | 初始化编码器 |
| `ENC_Update()` | 主循环更新，返回按键事件 |
| `ENC_NextTarget()` | 循环切换控制目标 |
| `ENC_SetTarget()` | 直接设置控制目标 |
| `ENC_GetTarget()` | 获取当前控制目标 |
| `ENC_ApplyToDDS()` | 将编码器值同步到DDS |
| `ENC_GetGear()` | 获取当前旋转速度档位 |

---

## 5. store — Flash预设存储

### 5.1 功能概述

将DDS和FFT配置保存到Flash（Bank2 Sector7, 0x081E0000），最多10套预设。上电自动加载默认预设。

### 5.2 初始化

```c
#include "store.h"

CFG_Init();   // 上电调用，自动加载默认预设
```

### 5.3 保存预设

```c
DDS_Config dds_cfg;
FFT_PresetCfg fft_cfg;

// 填充配置...
dds_cfg.algo = DDS_ALGO_FAST;
dds_cfg.wave = DDS_SINE;
fft_cfg.mode = FFT_MODE_PRECISE;
fft_cfg.window = FFT_WIN_HANNING;

// 保存到槽位0，名称"Preset1"
CFG_Save(0, "Preset1", &dds_cfg, &fft_cfg);
```

### 5.4 加载预设

```c
DDS_Config dds_cfg;
FFT_PresetCfg fft_cfg;

// 从槽位2加载
if (CFG_Load(2, &dds_cfg, &fft_cfg) == 0) {
    DDS_ApplyConfig(&dds1, &dds_cfg);
    MYFFT_SetMode(fft_handle, fft_cfg.mode);
}
```

### 5.5 管理操作

```c
CFG_Delete(0);           // 删除槽位0的预设
CFG_SetDefault(0);       // 设置槽位0为上电默认
CFG_Format();            // 格式化整个配置区（慎用！）

// 查看预设信息
char name[12];
CFG_GetPresetInfo(0, name);  // 获取槽位0的名称

// 活跃配置（RAM中的当前配置，不涉及Flash读写）
CFG_GetActive(&dds_cfg, &fft_cfg);
CFG_SetActive(&dds_cfg, &fft_cfg);  // 只改RAM，不写Flash
```

### 5.6 API快速索引

| 函数 | 功能 |
|------|------|
| `CFG_Init()` | 初始化，自动加载默认预设 |
| `CFG_Save()` | 保存预设到Flash |
| `CFG_Load()` | 从Flash加载预设 |
| `CFG_Delete()` | 删除指定预设 |
| `CFG_SetDefault()` | 设置默认预设槽位 |
| `CFG_Format()` | 格式化整个配置区 |
| `CFG_GetPresetInfo()` | 获取预设信息 |
| `CFG_GetActive()` | 获取RAM中的当前配置 |
| `CFG_SetActive()` | 设置RAM配置（不写Flash） |

---

## 6. 完整示例：mainoop.c 搭建流程

### 6.1 最简DDS输出

```c
#include "main.h"
#include "mydds.h"

static DDS_Handle dds1;

void main_init(void) {
    DDS_Init(&dds1, &htim4, &hdac1, DAC_CHANNEL_1);
    DDS_SetWave(&dds1, DDS_SINE, 1000.0f, 2.0f, 0.0f);
    // PA4输出1kHz正弦波，2V P-P
}

void main_loop(void) {}
```

### 6.2 DDS + 编码器调频

```c
#include "main.h"
#include "mydds.h"
#include "encoder.h"

static DDS_Handle dds1;
static ENC_Handle enc1;

void main_init(void) {
    DDS_Init(&dds1, &htim4, &hdac1, DAC_CHANNEL_1);
    DDS_SetWave(&dds1, DDS_SINE, 1000.0f, 2.0f, 0.0f);
    ENC_Init(&enc1, &htim1, KEY_GPIO_Port, KEY_Pin);
}

void main_loop(void) {
    ENC_Event_t evt = ENC_Update(&enc1);
    if (evt == ENC_EVT_CLICK) ENC_NextTarget(&enc1);
    if (enc1.rotated) {
        enc1.rotated = 0;
        ENC_ApplyToDDS(&enc1, &dds1);
    }
}
```

### 6.3 DDS + 波形显示 + FFT分析（项目10完整版）

```c
#include "main.h"
#include "oled.h"
#include "mydds.h"
#include "myfft.h"
#include "mydraw.h"
#include "encoder.h"
#include <stdio.h>

// ---- 波形显示 (ADC1, TIM2触发) ----
#define WAVE_BUF_LEN   128
#define WAVE_SAMPLE_HZ 64000
static uint16_t wave_buf[WAVE_BUF_LEN * 2];
static volatile uint8_t wave_ready = 0, wave_half = 0;

// ---- FFT分析 (ADC2, TIM3触发) ----
#define FFT_SAMPLE_HZ  64000
#define FFT_ADC_BUF_LEN (1024 * 2)
static uint16_t fft_adc_buf[FFT_ADC_BUF_LEN];
static volatile uint8_t fft_ready = 0, fft_half = 0;
static FFT_Handle *fft_handle;
static FFT_Result  fft_result;

// DDS + 编码器
static DDS_Handle dds1;
static ENC_Handle enc1;

// 定时器频率设置
static void TIM_Set_Frequency(TIM_HandleTypeDef *htim, uint32_t freq_hz) {
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SET_AUTORELOAD(htim, timer_clk / freq_hz - 1);
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
}

void main_init(void) {
    HAL_Delay(20);
    OLED_Init();

    // DDS: TIM4+DAC1_CH1(PA4), 1kHz正弦, 2V P-P
    DDS_Config dcfg = {
        .algo = DDS_ALGO_FAST, .wave = DDS_SINE,
        .table_size = 256, .target_rate = 10000000UL,
        .auto_switch_freq = 0, .enable_interp = 1
    };
    DDS_InitWithConfig(&dds1, &dcfg, &htim4, &hdac1, DAC_CHANNEL_1);
    DDS_SetWave(&dds1, DDS_SINE, 1000.0f, 2.0f, 0.0f);

    // 编码器
    ENC_Init(&enc1, &htim1, KEY_GPIO_Port, KEY_Pin);

    // ADC1: TIM2触发, DMA循环, 128点双缓冲, 波形显示
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)wave_buf, WAVE_BUF_LEN * 2);
    HAL_TIM_Base_Start(&htim2);
    TIM_Set_Frequency(&htim2, WAVE_SAMPLE_HZ);

    // ADC2: TIM3触发, DMA循环, 1024点双缓冲, FFT分析
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)fft_adc_buf, FFT_ADC_BUF_LEN);
    HAL_TIM_Base_Start(&htim3);
    TIM_Set_Frequency(&htim3, FFT_SAMPLE_HZ);

    // FFT: 精准模式
    fft_handle = MYFFT_Init(FFT_SAMPLE_HZ, 3.3f, 12);
    MYFFT_SetMode(fft_handle, FFT_MODE_PRECISE);
}

void main_loop(void) {
    // 编码器
    ENC_Event_t evt = ENC_Update(&enc1);
    if (evt == ENC_EVT_CLICK) ENC_NextTarget(&enc1);
    if (enc1.rotated) {
        enc1.rotated = 0;
        ENC_ApplyToDDS(&enc1, &dds1);
    }

    // 波形显示
    if (wave_ready) {
        wave_ready = 0;
        uint16_t *data = (wave_half == 0) ? &wave_buf[0] : &wave_buf[WAVE_BUF_LEN];
        MYDRAW_DrawSingleWave(data, WAVE_BUF_LEN, 0);
    }

    // FFT分析
    if (fft_ready) {
        fft_ready = 0;
        uint16_t *data = (fft_half == 0) ? &fft_adc_buf[0] : &fft_adc_buf[1024];
        MYFFT_Process(fft_handle, data, &fft_result);

        char buf[17];
        sprintf(buf, "%s %5.0fHz  %d", FFT_WaveStr(fft_result.wave_type),
                fft_result.frequency, (int)enc1.target + 1);
        OLED_ShowString(1, 1, buf);
        sprintf(buf, "A:%4.2fV", fft_result.amplitude);
        OLED_ShowString(2, 1, buf);
    }
}

// DMA回调
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) { wave_half = 0; wave_ready = 1; }
    if (hadc->Instance == ADC2) { fft_half = 0; fft_ready = 1; }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) { wave_half = 1; wave_ready = 1; }
    if (hadc->Instance == ADC2) { fft_half = 1; fft_ready = 1; }
}
```

### 6.4 产生特定波形的写法

```c
// 1kHz正弦波，2V P-P
DDS_SetWave(&dds1, DDS_SINE, 1000.0f, 2.0f, 0.0f);

// 5kHz方波，1.5V P-P
DDS_SetWave(&dds1, DDS_SQUARE, 5000.0f, 1.5f, 0.0f);

// 500Hz三角波，3V P-P，90°相位
DDS_SetWave(&dds1, DDS_TRIANGLE, 500.0f, 3.0f, 90.0f);

// 只改频率，波形和幅度不变
DDS_SetFreq(&dds1, 2000.0f);

// 只改幅度
DDS_SetAmp(&dds1, 1.0f);

// 只改相位
DDS_SetPhase(&dds1, 180.0f);
```

### 6.5 DAC双通道同步输出

```c
// 两个DAC通道都使用同一个TIM4触发，保证相位对齐
HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
HAL_TIM_Base_Start(&htim4);

// DMA回调（双缓冲）
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    DDS_RT_FillHalf(&ddrt);  // 填前半缓冲
}
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    DDS_RT_FillOther(&ddrt);  // 填后半缓冲
}
```

> **重要**：两个通道共用一个TIM4触发源，采样率完全相同。这是保证双通道相位对齐的关键。

### 6.6 ADC1双通道采集（相位测量）

**CubeMX配置：**
```
ADC1 → NbrOfConversion: 2
  Rank 1: ADC_CHANNEL_5 (PB1) — DDS反馈/DAC输出回采
  Rank 2: ADC_CHANNEL_4 (PA4) — 外部输入信号
```

**数据排列：**
```
adc_phase_buf[0] = CH5_采样0
adc_phase_buf[1] = CH4_采样0  ← 这两个几乎同时采
adc_phase_buf[2] = CH5_采样1
adc_phase_buf[3] = CH4_采样1
...
```

---

## 7. 硬件资源映射表

### 7.1 项目10（multiconfiger10）

| 外设 | 引脚 | 功能 | 模块 |
|------|------|------|------|
| TIM1 | PE9, PE11 | 编码器旋转检测 | encoder |
| PA0 | KEY_Pin | 编码器按键 | encoder |
| TIM4 | - | DAC1_CH1触发 | mydds |
| DAC1_CH1 | PA4 | DDS波形输出 | mydds |
| TIM2 | - | ADC1采样触发 (64kHz) | mydraw |
| ADC1_CH5 | PB1 | 波形采集输入 | mydraw |
| DMA1_Stream0 | - | ADC1 DMA传输 | wave |
| TIM3 | - | ADC2采样触发 (64kHz) | myfft |
| ADC2_CH3 | PA6 | FFT分析输入 | myfft |
| DMA1_Stream2 | - | ADC2 DMA传输 | myfft |
| I2C1 | PB7(SDA), PB8(SCL) | OLED通信 | mydraw |
| Flash Bank2 Sector7 | 0x081E0000 | 预设存储 | store |

### 7.2 项目11（multiconfiger11，CPLL）

| 外设 | 引脚 | 功能 | 模块 |
|------|------|------|------|
| TIM1 | PE9, PE11 | 编码器旋转检测 | encoder |
| PA0 | KEY_Pin | 编码器按键 | encoder |
| TIM4 | - | DAC双通道触发 | mydds(CPLL) |
| DAC1_CH1 | PA4 | DDS输出CH1 | mydds |
| DAC1_CH2 | PA5 | DDS输出CH2（相位偏移） | mydds |
| COMP2 | - | 过零检测 | cpll |
| TIM2 | - | ADC1采样触发 | myfft |
| ADC1 | CH4(PA4)+CH5(PB1) | 双通道相位测量 | myfft |

### 7.3 项目12（multiconfiger12，APLL）

| 外设 | 引脚 | 功能 | 模块 |
|------|------|------|------|
| ADC2 | CH3(PA6) | 采样输入信号 | apll |
| TIM3 | - | ADC2触发 200kHz | apll |
| DAC1 | CH1(PA4) | 输出延迟信号 | apll |
| TIM4 | - | DAC触发 200kHz | apll |
| TIM1 | PE9, PE11 | 编码器旋转检测 | encoder |
| PA0 | KEY_Pin | 编码器按键（校准/切换） | encoder |
| DMA1_Stream2 | - | ADC2 DMA CIRCULAR | apll |
| DMA1_Stream1 | - | DAC DMA CIRCULAR | apll |

> **关键**：APLL中ADC和DAC必须用相同采样率（都是200kHz），这是天然锁相的前提。

---

## 8. 锁相方案：CPLL与APLL

### 8.1 为什么需要锁相

电赛波形题的核心要求：**输出信号与输入信号同频同相**（或固定相位差）。

传统PLL（CD4046硬件锁相环）需要外接器件，而数字方案全部在MCU内完成，更灵活。

目前实现了两种数字锁相方案，各有优劣：

| 特性 | CPLL（过零硬同步） | APLL（数字延迟线） |
|------|-------------------|-------------------|
| 项目 | 11_multiconfiger11 | 12_multiconfiger12 |
| 核心原理 | 每个过零→重置DDS相位 | ADC写入ring→延迟读取→DAC输出 |
| 锁相机制 | 相位硬对齐（reset） | 时间延迟（天然锁相） |
| 频率跟踪 | 零交叉测频+DDS步进更新 | 零交叉测频+延迟量自适应 |
| 波形来源 | DDS查表生成（纯净正弦） | 原始ADC数据（保持输入波形形状） |
| 稳定性 | 较好，但需调参 | **极稳**，同时钟天然锁死 |
| 相位调节 | phase_off2（已验证但需优化） | delay_f（已验证，平滑连续） |
| 输入要求 | 需要比较器（内部COMP2） | 只需ADC（更简单） |
| 适用场景 | 要求输出为标准正弦 | 要求输出保持输入波形形状 |

---

### 8.2 CPLL — 过零硬同步DDS

#### 8.2.1 原理

```
输入信号 → 比较器(COMP2) → 过零中断 → 重置DDS相位累加器
                                      → 零交叉测频 → 更新DDS步进
```

**核心思想**：每个输入信号过零时刻，把DDS的相位拉回0°。这样DDS输出的每个周期都从0°开始，天然与输入对齐。

**本质**：这是一个**相位硬对齐系统**（Phase Reset System），不是传统意义上的PLL。

#### 8.2.2 关键设计

1. **同步标志而非直接操作**：不在COMP中断里直接改phase_acc，设`cpll_sync_pending`标志，在DMA FillHalf/FillOther边界同步（防波形撕裂）

2. **原子更新**：频率step也在DMA回调里通过`_cpll_apply`原子更新，和相位同步在同一时钟域

3. **短缓冲降延迟**：BUF_LEN从128降到16，半帧从320μs降到40μs，1kHz下相位误差从115°降到14°

4. **步进平滑**：`smooth_step = 75%旧 + 25%新`，防频率微抖

5. **窗口过滤**：delta在`period_cyc * 0.8~1.2`范围外的不接受，非法边沿不更新

#### 8.2.3 代码结构

```
mydds.c/h:
  DDS_RT_Handle    // DDS实时句柄（双通道+DMA双缓冲）
  DDS_RT_Init()    // 初始化
  DDS_RT_SetPhase2()  // 设置ch2相位偏移
  DDS_RT_FillHalf()   // 填前半帧（DMA HalfCplt回调）
  DDS_RT_FillOther()  // 填后半帧（DMA Cplt回调）
  _cpll_apply()       // CPLL原子更新：sync+step在同一边界

cpll.c/h:
  CPLL_OnEdge()    // 比较器过零回调：测频+设sync标志
```

#### 8.2.4 已知问题

- **同频时必须同时reset phase_acc1和phase_acc2**：否则ch2的phase_off2无效（已修复）
- 相位调节（旋转编码器）还需要进一步优化稳定性

#### 8.2.5 API快速索引

| 函数 | 功能 |
|------|------|
| `DDS_RT_Init()` | 初始化DDS实时句柄（双通道+DMA双缓冲） |
| `DDS_RT_SetPhase2()` | 设置CH2相位偏移 |
| `DDS_RT_FillHalf()` | 填前半帧（DMA HalfCplt回调） |
| `DDS_RT_FillOther()` | 填后半帧（DMA Cplt回调） |
| `CPLL_OnEdge()` | 比较器过零回调 |

---

### 8.3 APLL — 数字延迟线锁相

#### 8.3.1 原理

```
输入信号 → ADC采样 → 写入环形缓冲(ring)
                         ↓
                    从ring[widx - delay]读取
                         ↓
                    DAC输出 = 输入(t - τ)
```

**核心思想**：ADC和DAC用**同一个时钟**（都是200kHz），延迟τ是绝对时间。只要τ固定，输出相位永远固定，频率自动一致。

**本质**：这是一个**纯时间延迟系统**（Time Delay Line），比PLL更简单更稳——因为根本不需要"锁"，物理上就是同频的。

#### 8.3.2 关键设计

1. **浮点延迟**：`delay_f`是float，支持亚采样精度（如延迟50.3个样本），消除整数台阶

2. **线性插值读取**：从ring中取相邻两个采样点插值，波形平滑无台阶

3. **延迟平滑**：`delay_f += 0.05 * (target - delay)`，防止相位突变导致的波形毛刺

4. **在线零交叉测频**：上升沿检测→算spp（每周期采样点数）→自动计算`delay_target = phase_deg / 360 * spp`

5. **单样本处理**：`APLL_Step()`每个ADC采样调用一次，逻辑清晰

#### 8.3.3 APLL_Step 单样本处理流程

```
1. 写入 ring[widx] = adc          // 保存原始数据
2. 零交叉检测（上升沿）             // 更新spp
3. delay_target = phase/360 * spp  // 目标延迟
4. delay_f += 0.05*(target-delay)  // 平滑
5. ridx = widx - delay_f           // 读取位置（浮点）
6. i0 = (int)ridx, i1 = i0+1      // 相邻两点
7. v = ring[i0]*(1-frac) + ring[i1]*frac  // 线性插值
8. widx++                          // 推进指针
9. return v                        // DAC输出
```

#### 8.3.4 代码结构

```
apll.c/h:
  APLL_Handle      // 句柄（ring缓冲+延迟参数+零交叉状态）
  APLL_Init()      // 初始化（fs, f_init）
  APLL_SetPhase()  // 设置目标相位(度)和幅度
  APLL_Step()      // 单样本处理（核心）
  APLL_Process()   // 半帧批量处理
  APLL_Calibrate() // 校准固有延迟
  APLL_GetFreq()   // 获取测到的输入频率

mainoop.c:
  ADC2 DMA回调 → APLL_Process → DCache清理
```

#### 8.3.5 参数说明

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `delay_alpha` | 0.05 | 延迟平滑系数 | 小=更稳但响应慢，大=响应快但可能抖 |
| `spp低通` | 0.9/0.1 | 零交叉测频低通 | 0.9越大越稳，0.1越大响应越快 |
| `RING_SIZE` | 2048 | 环形缓冲大小 | ≥最大周期×2，1kHz@200kHz需≥400 |
| `amp_scale` | 1.0 | 输出幅度缩放 | 0~2，1=保持原幅度，编码器可调 |

#### 8.3.6 外设配置（项目12）

| 外设 | 配置 | 用途 |
|------|------|------|
| ADC2 | CH3(PA6), TIM3触发, DMA1_Stream2 CIRCULAR | 采样输入信号 |
| TIM3 | ARR=999 → 200kHz | ADC2触发 |
| DAC1 | CH1(PA4), TIM4触发, DMA1_Stream1 CIRCULAR | 输出延迟信号 |
| TIM4 | ARR=999 → 200kHz | DAC触发 |
| TIM1 | 编码器(PE9/PE11) | 旋转编码器（调相位/幅度） |
| PA0 | KEY(GPIO输入) | 编码器按键（校准/切换目标） |

> **踩坑**：DAC的`SampleAndHold`必须设为`DISABLE`，否则输出波形严重变形。

#### 8.3.7 API快速索引

| 函数 | 功能 |
|------|------|
| `APLL_Init()` | 初始化（采样率, 初始频率） |
| `APLL_SetPhase()` | 设置目标相位(度)和幅度缩放 |
| `APLL_Step()` | 单样本处理（核心） |
| `APLL_Process()` | 半帧批量处理 |
| `APLL_Calibrate()` | 校准固有延迟（按PA0触发） |
| `APLL_GetFreq()` | 获取测到的输入频率 |

---

### 8.4 两种方案的深层对比

#### 为什么APLL更稳？

CPLL是**离散事件驱动**：每个过零做一次硬同步，同步之间的时段内DDS自由跑。如果两次同步之间频率有微小漂移，输出会在下次同步时被硬拉回来——这就是"抖"的来源。

APLL是**连续时间系统**：ADC和DAC共享同一个时钟源，延迟是绝对的物理量。不存在"追"和"拉"的过程，所以天然不抖。

#### 什么时候用CPLL？

- 输出必须是**标准正弦**（输入波形不理想时）
- 需要**倍频输出**（APLL只能同频）
- 输入信号**幅度不稳**（比较器比ADC零交叉更鲁棒）

#### 什么时候用APLL？

- 要求**输出保持输入波形形状**（方波→方波，三角→三角）
- 要求**极稳的相位锁定**
- 不想用比较器，只靠ADC

---

### 8.5 APLL操作说明（项目12）

#### 编码器操作

| 操作 | 功能 | 说明 |
|------|------|------|
| 旋转（PHA模式） | 调相位 | 10°步进，0°~350° |
| 旋转（AMP模式） | 调幅度 | 0.1步进，0~2.0 |
| 短按(PA0) | 校准 | 当前位置设为0°参考 |
| 长按(PA0) | 切换目标 | PHA ↔ AMP |

#### 相位校准流程

由于DMA半帧缓冲等系统延迟，上电后P=0°时输出与输入不一定是0°相位差。需要手动校准：

1. 上电后，旋转编码器移到示波器上两波重合的位置
2. 短按PA0按键 → `APLL_Calibrate`记录当前delay_f为0°参考
3. 之后P=0°=同相，P=180°=反相

校准原理：
```c
// 按键校准时记录当前位置
h->inherent_offset = h->delay_f;

// 之后每样本计算目标延迟
h->delay_target = h->inherent_offset + h->target_phase_deg / 360.0f * h->spp;
```

#### OLED显示

| 行 | 内容 | 示例 |
|----|------|------|
| 第1行 | 锁定状态+频率 | `LK 1000.0Hz` / `.. 0.0Hz` |
| 第2行 | 当前目标+相位+幅度 | `PH 90 A1.0` / `AM 1.5 PH90` |

- `PH` = 当前在调相位，数字是当前相位°
- `AM` = 当前在调幅度，数字是当前幅度倍率
- `LK` = 已锁定，`..` = 未锁定

---

### 8.6 踩坑记录

#### APLL迭代过程中踩过的坑

| 问题 | 原因 | 解决 |
|------|------|------|
| DAC无输出 | DAC的SampleAndHold=ENABLE | 改为DISABLE |
| "一段正弦一段平" | 半帧回调只填dac_buf[0..127]，后半段是旧数据 | APLL_Process加dac_offset参数 |
| 波形混乱+频率不稳 | 每半帧独立重建波形模板，相位不连续 | 换成连续相位推进(phase_acc) |
| 仍然混乱 | 每帧重建模板+phase_acc连续推进=矛盾 | 彻底换架构：环形缓冲延迟线 |
| I/Q锁相环输出不像原波形 | NCO重建正弦，不是原信号 | 回归延迟线方案 |
| 相位台阶状 | 延迟取整数采样点 | 浮点延迟+线性插值 |
| 延迟突变导致毛刺 | delay_f直接跳到target | delay_f += alpha*(target-delay)平滑 |
| P=228°才同相 | 系统固有DMA延迟未补偿 | 手动校准：APLL_Calibrate记录inherent_offset |
| 硬编码校准值不准 | 不同频率/配置下固有延迟不同 | 改为手动校准方案，每次上电按需校 |

#### CPLL踩过的坑

| 问题 | 原因 | 解决 |
|------|------|------|
| 相位偏移无效 | _cpll_apply只清phase_acc1不清phase_acc2 | 同时清零phase_acc2 |
| 波形撕裂 | COMP中断里直接改phase_acc | 改为设cpll_sync_pending标志，DMA边界同步 |
| 同步延迟大 | BUF_LEN=128，半帧320μs | 降到16，半帧40μs |
| 频率微抖 | step直接用新值 | 75%/25%低通平滑 |

---

## 9. 常见问题

### Q: FFT报告的频率跟DDS输出不一致？
A: FFT频率分辨率 = 采样率/点数 = 64000/1024 = 62.5Hz。只有频率恰好是62.5Hz整数倍时才精确，其余会被"吸"到最近bin。例如DDS输出1040Hz，FFT报告1062Hz，这是正常的量化效应。

### Q: 旋转编码器切波形时屏幕卡住？
A: 确保`ENC_ApplyToDDS`只在`enc1.rotated`为1时调用，不要每帧都调。每帧都调`DDS_SetWave`会重建波形表，极慢。

### Q: 波形显示和FFT文字互相覆盖？
A: 当前实现是先画波形再写文字，文字叠在波形上方。如果需要分屏或分页显示，后续可以改。

### Q: 怎么改ADC采样率？
A: 调用`TIM_Set_Frequency(&htim2, 新频率)`改ADC1，`TIM_Set_Frequency(&htim3, 新频率)`改ADC2。同时要用`MYFFT_SetSampleRate`更新FFT内部的采样率数值，否则频率计算会错。

### Q: DDS输出频率上限是多少？
A: FAST算法（256点查表）建议10kHz以下；PRECISE算法（相位累加）可到100kHz。用`auto_switch_freq`参数可以设置自动切换阈值。

### Q: 怎么同时输出两路DDS？
A: 声明第二个`DDS_Handle`，绑定TIM5+DAC1_CH2(PA5)：
```c
static DDS_Handle dds2;
DDS_Init(&dds2, &htim5, &hdac1, DAC_CHANNEL_2);
DDS_SetWave(&dds2, DDS_SINE, 2000.0f, 1.0f, 90.0f);
```

### Q: Flash预设存储会擦坏吗？
A: Flash擦写寿命约10万次。`CFG_Save`先擦后写，频繁保存会损耗。建议调试时用`CFG_SetActive`（只改RAM），确认无误后再`CFG_Save`写Flash。

### Q: APLL校准后换频率需要重新校准吗？
A: 一般不需要。`inherent_offset`是系统固有的DMA延迟，与频率无关。`spp`（每周期采样点数）会随频率自动变化，`delay_target = inherent_offset + phase/360*spp`会自动适配。但如果有明显偏差，重新校准一次即可。

### Q: CPLL和APLL能同时用吗？
A: 不建议。两者共用ADC2和DAC1资源，同时运行会冲突。根据需求选一个。

### Q: APLL能输出倍频信号吗？
A: 不能。APLL本质是延迟线，输出频率与输入相同。需要倍频请用CPLL方案，在DDS步进里设倍频系数。
