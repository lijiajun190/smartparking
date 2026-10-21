/**
 * @file    bsp_sensors.h
 * @brief   传感器驱动: 红外对管、限位开关、急停按钮
 * @note    硬件说明(全部为上拉输入 + 双边沿 EXTI):
 *          | 逻辑名           | 引脚 | 中断线      | 触发含义                |
 *          |------------------|------|-------------|-------------------------|
 *          | IR_SENSOR_1      | PC5  | EXTI9_5     | 车位1 有车遮挡          |
 *          | IR_SENSOR_2      | PC6  | EXTI9_5     | 车位2 有车遮挡          |
 *          | LIMIT_SW_1       | PC7  | EXTI9_5     | 载车板到达 1 号限位     |
 *          | LIMIT_SW_2       | PC8  | EXTI9_5     | 载车板到达 2 号限位     |
 *          | EMERGENCY_STOP   | PB2  | EXTI2       | 急停按钮按下            |
 *
 *          注意: PC5~PC8 共用 EXTI9_5_IRQHandler, 中断里必须靠引脚号区分。
 *
 *          设计思路:
 *          1. 电平读取(sensor_read)是纯查询, 任何任务都能安全调用。
 *          2. EXTI 中断锁存急停/方向限位，记录变化位图并通知任务。
 *             不输出电机相位，也不在中断里跑业务逻辑,
 *             这是 CLAUDE.md "中断最小化"原则的落地。
 *          3. 位图 + 任务通知的组合可以保证事件不丢: 即使 SafetyTask
 *             还没来得及处理, 后续中断只是往位图里再置一位。
 *          4. 急停单独提供 sensor_emergency_active(), 因为它优先级最高,
 *             SafetyTask 会直接轮询它而不依赖中断。
 */
#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 传感器编号                                                                */
/* ------------------------------------------------------------------------- */
typedef enum
{
    SENSOR_IR_1 = 0,        /* PC5  红外传感器 1 */
    SENSOR_IR_2,            /* PC6  红外传感器 2 */
    SENSOR_LIMIT_1,         /* PC7  限位开关 1 */
    SENSOR_LIMIT_2,         /* PC8  限位开关 2 */
    SENSOR_EMERGENCY,       /* PB2  急停按钮 */
    SENSOR_ID_COUNT         /* 传感器总数, 必须放最后 */
} SensorId;

/* 传感器变化位图: 第 n 位对应 SensorId 为 n 的传感器 */
#define SENSOR_FLAG(id)     (1UL << (uint32_t)(id))

/**
 * @brief  传感器模块初始化
 * @note   GPIO 与 EXTI 的配置由 CubeMX 的 MX_GPIO_Init() 完成,
 *         本函数只清空变化位图并采样一次初始电平。
 * @retval HAL_OK 成功
 */
HAL_StatusTypeDef sensor_init(void);

/**
 * @brief  读取传感器当前是否处于"触发"状态
 * @param  sensor_id 传感器编号
 * @note   已把高/低电平有效的差异屏蔽掉, 返回值统一是逻辑含义:
 *         红外 = 有遮挡, 限位 = 已到位, 急停 = 已按下。
 * @retval 1 = 触发; 0 = 未触发; 非法编号返回 0
 */
uint8_t sensor_read(SensorId sensor_id);

/**
 * @brief  一次性读取全部传感器状态, 打包成位图
 * @note   适合 UiTask 刷新显示或 SafetyTask 做整体判断, 只需一次调用。
 * @retval 位图, 第 n 位为 1 表示编号 n 的传感器处于触发状态
 */
uint32_t sensor_read_all(void);

/**
 * @brief  急停是否处于按下状态(便捷封装)
 * @retval 1 = 急停按下; 0 = 正常
 */
uint8_t sensor_emergency_active(void);

/**
 * @brief  注册"传感器变化"通知的目标任务
 * @param  task_handle FreeRTOS 任务句柄(TaskHandle_t), 传 NULL 表示取消注册
 * @note   传感器 EXTI 触发后, 驱动会用 vTaskNotifyGiveFromISR() 通知该任务。
 *         通常由 SafetyTask 在自己启动时注册自己。
 *         参数类型用 void* 是为了让本头文件不必包含 FreeRTOS 头,
 *         保持 BSP 层对 RTOS 的弱依赖。
 */
void sensor_set_notify_task(void *task_handle);

/**
 * @brief  取出并清空"发生变化的传感器"位图
 * @note   读取即清零(read-and-clear), 保证同一个变化不会被处理两次。
 *         内部通过关中断保护, 可安全对抗 EXTI 并发。
 * @retval 位图, 第 n 位为 1 表示编号 n 的传感器自上次调用后发生过跳变
 */
uint32_t sensor_take_changed_flags(void);

/**
 * @brief  EXTI 回调的内部处理函数
 * @param  gpio_pin 触发中断的引脚号
 * @note   由 HAL_GPIO_EXTI_Callback() 调用, 业务代码不要直接调用。
 *         HAL_GPIO_EXTI_Callback() 的实现放在本文件中。
 */
void sensor_exti_handler(uint16_t gpio_pin);

#endif /* BSP_SENSORS_H */
