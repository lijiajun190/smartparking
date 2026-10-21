/**
 * @file    bsp_keys.c
 * @brief   按键扫描与消抖驱动实现
 * @note    消抖状态机说明:
 *
 *          raw(原始电平) ──┐
 *                          ├─> 与 stable 不同? ─是─> counter++
 *                          │                          counter 达到阈值 ──> 更新 stable, 产生事件
 *                          └─> 相同? ────────────────> counter 清零
 *
 *          抖动期间原始电平反复跳变, counter 会被不断清零, 因此凑不满阈值,
 *          只有电平真正稳定 20ms 之后才会确认, 这就是消抖的原理。
 */

#include "bsp_keys.h"

/* ------------------------------------------------------------------------- */
/* 按键硬件描述表                                                            */
/* 把"编号 -> 引脚"的映射集中到一张表里, 加按键时只改这张表 + KeyId 枚举   */
/* ------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *port;         /* GPIO 端口 */
    uint16_t      pin;          /* GPIO 引脚 */
    GPIO_PinState active_level;  /* 按下时的电平: 上拉+接地按键为低电平有效 */
} KeyHwConfig;

static const KeyHwConfig s_key_hw[KEY_ID_COUNT] =
{
    /* KEY_ID_USER: PA0, 上拉输入, 按下接地 ==> 低电平有效 */
    { KEY_USER_GPIO_Port, KEY_USER_Pin, GPIO_PIN_RESET }
};

/* ------------------------------------------------------------------------- */
/* 按键运行时状态                                                            */
/* ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  stable_pressed;    /* 已消抖确认的状态: 1=按下, 0=松开 */
    uint8_t  debounce_count;    /* 当前候选电平的连续采样计数 */
    uint16_t hold_count;        /* 按下保持的扫描次数, 用于长按判定 */
    uint8_t  long_reported;     /* 本次按下是否已上报过长按(防止重复上报) */
} KeyState;

static KeyState s_key_state[KEY_ID_COUNT];

/**
 * @brief  读取按键原始电平并归一化为"是否按下"
 * @param  key_id 按键编号
 * @retval 1 = 按下; 0 = 松开
 */
static uint8_t key_read_raw(KeyId key_id)
{
    GPIO_PinState level;

    level = HAL_GPIO_ReadPin(s_key_hw[key_id].port, s_key_hw[key_id].pin);

    /* 与"有效电平"比较, 屏蔽掉高/低有效的差异,
     * 这样上层逻辑永远只关心"按下/松开" */
    return (level == s_key_hw[key_id].active_level) ? 1U : 0U;
}

/**
 * @brief  按键模块初始化
 */
HAL_StatusTypeDef key_init(void)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)KEY_ID_COUNT; i++)
    {
        /* 用当前真实电平作为初始稳定状态。
         * 这样做的好处: 如果上电时按键就被按住, 不会误报一次 PRESS 事件。 */
        s_key_state[i].stable_pressed = key_read_raw((KeyId)i);
        s_key_state[i].debounce_count = 0;
        s_key_state[i].hold_count     = 0;
        s_key_state[i].long_reported  = 0;
    }

    return HAL_OK;
}

/**
 * @brief  扫描单个按键
 */
KeyEvent key_scan(KeyId key_id)
{
    uint8_t   raw;
    KeyState *st;

    if (key_id >= KEY_ID_COUNT)
    {
        return KEY_EVENT_NONE;      /* 非法编号, 直接忽略 */
    }

    st  = &s_key_state[key_id];
    raw = key_read_raw(key_id);

    /* ---------- 第一步: 消抖, 判断稳定状态是否发生变化 ---------- */
    if (raw != st->stable_pressed)
    {
        /* 原始电平与稳定状态不一致, 累计候选计数 */
        st->debounce_count++;

        if (st->debounce_count >= KEY_DEBOUNCE_COUNT)
        {
            /* 连续 KEY_DEBOUNCE_COUNT 次都不一致 ==> 确认状态翻转 */
            st->stable_pressed = raw;
            st->debounce_count = 0;

            if (raw != 0U)
            {
                /* 松开 -> 按下: 重置长按计时并上报 PRESS */
                st->hold_count    = 0;
                st->long_reported = 0;
                return KEY_EVENT_PRESS;
            }
            else
            {
                /* 按下 -> 松开: 上报 RELEASE */
                st->hold_count = 0;
                return KEY_EVENT_RELEASE;
            }
        }

        /* 消抖窗口未满, 本次不产生事件 */
        return KEY_EVENT_NONE;
    }

    /* 原始电平与稳定状态一致, 说明没在抖动, 计数清零 */
    st->debounce_count = 0;

    /* ---------- 第二步: 持续按下时的长按判定 ---------- */
    if (st->stable_pressed != 0U)
    {
        /* hold_count 做饱和处理, 避免长时间按住导致溢出回绕 */
        if (st->hold_count < 0xFFFFU)
        {
            st->hold_count++;
        }

        if ((st->long_reported == 0U) &&
            (st->hold_count >= KEY_LONG_PRESS_COUNT))
        {
            st->long_reported = 1;      /* 一次按下只上报一次长按 */
            return KEY_EVENT_LONG_PRESS;
        }
    }

    return KEY_EVENT_NONE;
}

/**
 * @brief  扫描全部按键
 */
KeyEvent key_scan_all(KeyId *out_key_id)
{
    uint8_t  i;
    KeyEvent event;

    for (i = 0; i < (uint8_t)KEY_ID_COUNT; i++)
    {
        event = key_scan((KeyId)i);

        if (event != KEY_EVENT_NONE)
        {
            if (out_key_id != NULL)
            {
                *out_key_id = (KeyId)i;
            }
            return event;
        }
    }

    return KEY_EVENT_NONE;
}

/**
 * @brief  查询已消抖的按下状态
 */
uint8_t key_is_pressed(KeyId key_id)
{
    if (key_id >= KEY_ID_COUNT)
    {
        return 0;
    }

    return s_key_state[key_id].stable_pressed;
}
