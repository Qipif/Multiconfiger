/**
 * competition5 - 程控移相器 UI
 *
 * 四个模式，右上角显示模式号：
 *   模式0：校准界面 - 旋转编码器移相，按确认键校准为180°
 *   模式1：移相界面 - 旋转编码器移相(2°步进，0~180°)，确认键无效
 *   模式2：相位设置 - 旋转编码器改预设值(1°步进)，按确认键写入
 *   模式3：自动循环 - 旋转编码器改周期(0.5s步进)，确认键无效
 *
 * 按键：
 *   PA0 = 模式切换（EXTI0中断驱动）
 *   PB2 = 确认按键（EXTI2中断驱动）
 *
 * 参考08_multiconfiger8的架构：状态机+OLED分时刷新
 */

#include "main.h"
#include "oled.h"
#include "edgesync.h"
#include <stdio.h>

// ── 模式 ──
typedef enum {
    MODE_CALIBRATE = 0,  // 校准
    MODE_PHASE,          // 移相
    MODE_SET,            // 相位设置
    MODE_AUTO,           // 自动循环
    MODE_COUNT           // 模式总数
} AppMode;

// ── 按键事件（EXTI ISR设flag，主循环消费） ──
volatile uint8_t  btn_mode_pressed = 0;    // PA0 模式切换
volatile uint32_t btn_mode_timestamp = 0;
volatile uint8_t  btn_ok_pressed = 0;      // PB2 确认
volatile uint32_t btn_ok_timestamp = 0;

#define BTN_DEBOUNCE_MS  50  // 消抖时间（ISR端30ms+主循环端50ms双保险）

// ── 状态 ──
static AppMode cur_mode = MODE_CALIBRATE;

// 各模式参数
static float phase_val  = 0.0f;   // 当前相位（模式0/1用）
static float set_val    = 0.0f;   // 预设值（模式2用）
static float auto_time  = 15.0f;  // 自动循环周期（模式3用）

// 自动循环状态
static uint8_t auto_running = 0;
static float    auto_phase  = 0.0f;
static uint32_t auto_last_tick = 0;

// ── 初始化 ──
void main_init(void)
{
    // DWT计数器（EdgeSync测频）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    OLED_Init();

    EdgeSync_Init();
    EdgeSync_SetAmp(2.7f);

    // 停掉不需要的定时器
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_Base_Stop(&htim4);

    // 编码器
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // 初始显示
    OLED_ShowString(1, 1, "PHASE SHIFT  0");
    OLED_ShowString(2, 1, "Ph: 0  CAL:");
    OLED_ShowString(3, 1, "              ");
}

// ── 编码器旋转检测（简化版，直接读delta） ──
static int16_t encoder_get_delta(void)
{
    static int16_t last_cnt = 0;
    int16_t cur = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t diff = cur - last_cnt;
    last_cnt = cur;

    // 4倍频除以4
    static int32_t remain = 0;
    remain += diff;
    int16_t delta = remain / 4;
    remain %= 4;
    return delta;
}

// ── 按键消抖处理 ──
static uint32_t btn_mode_last = 0;
static uint32_t btn_ok_last = 0;

static uint8_t consume_btn(volatile uint8_t *flag, uint32_t *last_time)
{
    if (*flag) {
        *flag = 0;
        uint32_t now = HAL_GetTick();
        if (now - *last_time < BTN_DEBOUNCE_MS) {
            return 0;  // 消抖
        }
        *last_time = now;
        return 1;
    }
    return 0;
}

// ── 主循环 ──
void main_loop(void)
{
    uint32_t now = HAL_GetTick();

    // ── 编码器旋转 ──
    int16_t delta = encoder_get_delta();
    if (delta != 0) {
        switch (cur_mode) {
        case MODE_CALIBRATE:
        case MODE_PHASE:
            // 2°步进，0~180°
            phase_val += delta * 2.0f;
            if (phase_val < 0.0f)   phase_val = 0.0f;
            if (phase_val > 180.0f) phase_val = 180.0f;
            EdgeSync_SetPhase(phase_val);
            break;

        case MODE_SET:
            // 1°步进
            set_val += delta * 1.0f;
            if (set_val < 0.0f)   set_val = 0.0f;
            if (set_val > 180.0f) set_val = 180.0f;
            break;

        case MODE_AUTO:
            // 0.5s步进
            auto_time += delta * 0.5f;
            if (auto_time < 1.0f)  auto_time = 1.0f;
            if (auto_time > 60.0f) auto_time = 60.0f;
            break;

        default: break;
        }
    }

    // ── PA0：模式切换 ──
    if (consume_btn(&btn_mode_pressed, &btn_mode_last)) {
        // 离开自动模式时停止
        if (cur_mode == MODE_AUTO) {
            auto_running = 0;
        }

        cur_mode = (AppMode)((cur_mode + 1) % MODE_COUNT);

        // 进入新模式时同步参数
        switch (cur_mode) {
        case MODE_CALIBRATE:
        case MODE_PHASE:
            phase_val = EdgeSync_GetPhase();
            if (phase_val > 180.0f) phase_val = 180.0f;
            break;
        case MODE_SET:
            set_val = EdgeSync_GetPhase();
            if (set_val > 180.0f) set_val = 180.0f;
            break;
        case MODE_AUTO:
            auto_phase = EdgeSync_GetPhase();
            auto_running = 1;
            auto_last_tick = now;
            break;
        default: break;
        }
    }

    // ── PB2：确认键 ──
    if (consume_btn(&btn_ok_pressed, &btn_ok_last)) {
        switch (cur_mode) {
        case MODE_CALIBRATE:
            // 校准：当前相位记为180°（加冷却期防双击）
            {
                static uint32_t calib_last = 0;
                uint32_t calib_now = HAL_GetTick();
                if (calib_now - calib_last >= 500) {
                    calib_last = calib_now;
                    EdgeSync_Calibrate(phase_val);
                    phase_val = 180.0f;
                    EdgeSync_SetPhase(180.0f);
                }
            }
            break;

        case MODE_PHASE:
            // 确认键无效
            break;

        case MODE_SET:
            // 写入预设值到相位
            EdgeSync_SetPhase(set_val);
            phase_val = set_val;
            break;

        case MODE_AUTO:
            // 确认键无效
            break;

        default: break;
        }
    }

    // ── 测频更新 ──
    EdgeSync_UpdateFreq();

    // ── 自动循环相位 ──
    if (cur_mode == MODE_AUTO && auto_running) {
        // 计算相位增量：0→180°需要 auto_time 秒
        // 每ms增加 180.0 / (auto_time * 1000) 度
        float phase_step = 180.0f / (auto_time * 1000.0f);
        uint32_t dt = now - auto_last_tick;
        auto_last_tick = now;

        auto_phase += phase_step * dt;
        if (auto_phase >= 180.0f) {
            auto_phase -= 180.0f;  // 循环：0→180→0→180...
        }
        EdgeSync_SetPhase(auto_phase);
    }

    // ── OLED显示（200ms刷新） ──
    static uint32_t disp_tick = 0;
    if (now - disp_tick < 200) return;
    disp_tick = now;

    char buf[17];
    float freq = EdgeSync_GetFreq();

    // 第1行：频率 + 右上角模式号
    sprintf(buf, "%s%.0fHz     %d",
            freq > 0 ? "" : "..",
            freq,
            cur_mode);
    OLED_ShowString(1, 1, buf);

    // 第2行：相位/预设/周期
    switch (cur_mode) {
    case MODE_CALIBRATE:
        sprintf(buf, "Ph:%-3.0f CAL:", phase_val);
        break;
    case MODE_PHASE:
        sprintf(buf, "Phase: %-6.0f ", phase_val);
        break;
    case MODE_SET:
        sprintf(buf, "Ph:%-3.0f Set:%-3.0f",
                EdgeSync_GetPhase(), set_val);
        break;
    case MODE_AUTO:
        sprintf(buf, "Ph:%-3.0f T:%-4.1fs",
                EdgeSync_GetPhase(), auto_time);
        break;
    default:
        sprintf(buf, "               ");
        break;
    }
    OLED_ShowString(2, 1, buf);

    // 第3行：模式提示
    switch (cur_mode) {
    case MODE_CALIBRATE:
        sprintf(buf, "OK=calib 180  ");
        break;
    case MODE_PHASE:
        sprintf(buf, "Step:2  0~180 ");
        break;
    case MODE_SET:
        sprintf(buf, "OK=write Set  ");
        break;
    case MODE_AUTO:
        sprintf(buf, "Auto %s T=%.1f ",
                auto_running ? "ON " : "OFF",
                auto_time);
        break;
    default:
        sprintf(buf, "               ");
        break;
    }
    OLED_ShowString(3, 1, buf);
}
