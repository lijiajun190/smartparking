/**
 * @file    bsp_keys.h
 * @brief   按键扫描与消抖驱动
 * @note    硬件说明:
 *          - PA0 = KEY_USER, CubeMX 配置为 GPIO_Input + 上拉
 *          - 按键一端接 PA0, 另一端接 GND ==> 按下读到低电平(0), 松开读到高电平(1)
 *
 *          设计思路(重要, 后续加按键照这个套路扩展即可):
 *          1. 采用"周期轮询 + 计数消抖", 不用 EXTI 中断。
 *             机械按键抖动可达 10ms 量级, 轮询消抖比中断更稳、代码更简单。
 *          2. 由 InputTask 每 10ms 调用一次 key_scan()。
 *             同一电平连续采到 KEY_DEBOUNCE_COUNT 次才确认状态变化,
 *             即 2 次 x 10ms = 20ms 消抖窗口(对应 CLAUDE.md 的 20ms 要求)。
 *          3. key_scan() 返回"事件", 而不是"电平":
 *             按下的瞬间返回一次 KEY_EVENT_PRESS, 之后一直按着也不再重复上报,
 *             这样业务层无需自己做边沿检测。
 *          4. 长按单独上报 KEY_EVENT_LONG_PRESS, 用于"管理员确认/复位"这类
 *             需要防误触的操作。
 */
#ifndef BSP_KEYS_H
#define BSP_KEYS_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 按键编号 —— 目前只有板载 PA0 一个,                                       */
/* 后续外接存车/取车/缴费/管理员按键时, 在 KEY_ID_USER 后面追加即可        */
/* ------------------------------------------------------------------------- */
typedef enum
{
    KEY_ID_USER = 0,        /* PA0 板载用户按键 */
    KEY_ID_COUNT            /* 按键总数, 必须放在最后 */
} KeyId;

/* ------------------------------------------------------------------------- */
/* 按键事件                                                                  */
/* ------------------------------------------------------------------------- */
typedef enum
{
    KEY_EVENT_NONE = 0,     /* 无事件(最常见的返回值) */
    KEY_EVENT_PRESS,        /* 按下瞬间, 只上报一次 */
    KEY_EVENT_RELEASE,      /* 松开瞬间, 只上报一次 */
    KEY_EVENT_LONG_PRESS    /* 按住超过长按阈值, 只上报一次 */
} KeyEvent;

/* ------------------------------------------------------------------------- */
/* 时间参数 —— 单位是"扫描次数", 需配合 KEY_SCAN_PERIOD_MS 换算            */
/* ------------------------------------------------------------------------- */
#define KEY_SCAN_PERIOD_MS      10U     /* 建议的 key_scan() 调用周期 */
#define KEY_DEBOUNCE_COUNT      2U      /* 消抖: 2 x 10ms = 20ms */
#define KEY_LONG_PRESS_COUNT    150U    /* 长按: 150 x 10ms = 1500ms */

/**
 * @brief  按键模块初始化
 * @note   PA0 的 GPIO 配置由 CubeMX 的 MX_GPIO_Init() 完成,
 *         本函数只清零内部消抖状态。必须在 MX_GPIO_Init() 之后调用。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef key_init(void);

/**
 * @brief  扫描一个按键, 返回本次扫描产生的事件
 * @param  key_id 按键编号, 见 KeyId
 * @note   必须周期调用(建议 KEY_SCAN_PERIOD_MS = 10ms), 消抖时间依赖调用周期。
 * @retval KeyEvent 本次扫描产生的事件; 无变化时返回 KEY_EVENT_NONE
 */
KeyEvent key_scan(KeyId key_id);

/**
 * @brief  扫描所有按键, 返回第一个产生事件的按键
 * @param  out_key_id [out] 产生事件的按键编号, 可传 NULL
 * @note   便于 InputTask 一次调用处理全部按键。
 *         同一次扫描有多个按键同时动作时, 只上报编号最小的那个,
 *         下一次扫描会上报剩下的(事件不会丢, 只是延后 10ms)。
 * @retval KeyEvent 事件类型; 全部无变化时返回 KEY_EVENT_NONE
 */
KeyEvent key_scan_all(KeyId *out_key_id);

/**
 * @brief  读取按键当前的稳定电平状态(已消抖)
 * @param  key_id 按键编号
 * @retval 1 = 当前处于按下状态; 0 = 当前处于松开状态
 */
uint8_t key_is_pressed(KeyId key_id);

#endif /* BSP_KEYS_H */
