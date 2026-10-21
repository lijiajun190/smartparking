/**
 * @file    bsp_buzzer.h
 * @brief   蜂鸣器驱动 (PA1 = TIM2_CH2 PWM)
 * @note    硬件说明:
 *          - 引脚: PA1, 复用功能 AF1 = TIM2_CH2
 *          - CubeMX 已配置 TIM2: Prescaler=99, Period=333, Pulse=166
 *            定时器时钟 100MHz / (99+1) = 1MHz
 *            PWM 频率 1MHz / (333+1) ≈ 3.0kHz, 占空比 166/334 ≈ 50%
 *          - 3kHz 在常见无源蜂鸣器的谐振频率附近, 声音最响
 *
 *          本驱动只负责"怎么发声", 不负责"什么时候发声"。
 *          业务层通过 buzzer_beep_xxx() 表达语义(提示/完成/错误),
 *          避免把频率、时长这些硬件细节泄漏到 App 层。
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 提示音时长定义 (单位: ms) —— 集中在这里方便统一调整手感                 */
/* ------------------------------------------------------------------------- */
#define BUZZER_MS_SHORT     50U     /* 短促提示音: 按键、刷卡确认      */
#define BUZZER_MS_DONE      200U    /* 完成音:     存/取车动作结束     */
#define BUZZER_MS_TICK      20U     /* 极短的"哒"声: 用于连续反馈      */

/**
 * @brief  蜂鸣器初始化
 * @note   必须在 MX_TIM2_Init() 之后调用。
 *         初始化完成后蜂鸣器处于静音状态(PWM 未启动)。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_init(void);

/**
 * @brief  持续发声(非阻塞, 立即返回)
 * @note   需要自己调用 buzzer_off() 停止, 用于故障长鸣等场合。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_on(void);

/**
 * @brief  停止发声(非阻塞)
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_off(void);

/**
 * @brief  设置发声频率
 * @param  freq_hz 目标频率, 有效范围 500~10000 Hz
 * @note   通过修改 TIM2 的自动重装载值实现, 占空比自动保持 50%。
 * @retval HAL_OK 成功; HAL_ERROR 频率超出范围
 */
HAL_StatusTypeDef buzzer_set_freq(uint32_t freq_hz);

/**
 * @brief  短提示音 (约 50ms) —— 阻塞
 * @note   使用 HAL_Delay 延时。本工程 HAL 时基是 TIM11(独立于 FreeRTOS),
 *         因此在任务中调用是安全的, 但会占用当前任务 50ms。
 *         禁止在中断中调用。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_beep_short(void);

/**
 * @brief  完成音 (约 200ms 长鸣) —— 阻塞
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_beep_done(void);

/**
 * @brief  错误音 (三声短促急鸣, 约 300ms) —— 阻塞
 * @note   故障需要持续报警时, 请改用 buzzer_on() 由 SafetyTask 控制关断。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_beep_error(void);

/**
 * @brief  自检: 发出一声短音, 用于确认硬件连通
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef buzzer_selftest(void);

#endif /* BSP_BUZZER_H */
