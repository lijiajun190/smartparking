/**
 * @file    bsp_sensors.c
 * @brief   传感器驱动实现: 红外对管、限位开关、急停按钮
 * @note    EXTI2、EXTI9_5 已使能；中断锁存急停/方向限位，任务轮询兜底。
 */

#include "bsp_sensors.h"
#include "bsp_stepper.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------------- */
/* 传感器硬件描述表                                                          */
/* 集中管理"编号 -> 引脚 + 有效电平", 新增传感器只改这张表和 SensorId 枚举  */
/* ------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *port;             /* GPIO 端口 */
    uint16_t      pin;              /* GPIO 引脚 */
    GPIO_PinState active_level;     /* "触发"对应的电平 */
} SensorHwConfig;

static const SensorHwConfig s_sensor_hw[SENSOR_ID_COUNT] =
{
    /* 红外对管模块 OUT 脚通常是: 有遮挡输出低电平 ==> 低电平有效 */
    { IR_SENSOR_1_GPIO_Port,    IR_SENSOR_1_Pin,    GPIO_PIN_RESET },
    { IR_SENSOR_2_GPIO_Port,    IR_SENSOR_2_Pin,    GPIO_PIN_RESET },

    /* 限位开关: 常开型, 一端接引脚一端接地, 压到时导通拉低 ==> 低电平有效 */
    { LIMIT_SW_1_GPIO_Port,     LIMIT_SW_1_Pin,     GPIO_PIN_RESET },
    { LIMIT_SW_2_GPIO_Port,     LIMIT_SW_2_Pin,     GPIO_PIN_RESET },

    /* 急停按钮: 上拉输入, 常开型, 按下接地 ==> 低电平有效 */
    { EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin, GPIO_PIN_RESET }
};

/* ------------------------------------------------------------------------- */
/* 模块内部状态                                                              */
/* ------------------------------------------------------------------------- */

/* 变化位图。volatile 是必须的: 它在中断里被写、在任务里被读,
 * 不加 volatile 编译器可能把读取优化成缓存值, 导致永远读不到更新。 */
static volatile uint32_t s_changed_flags = 0;

/* 需要被通知的任务句柄。同样在中断中使用, 加 volatile。 */
static TaskHandle_t volatile s_notify_task = NULL;

/**
 * @brief  传感器模块初始化
 */
HAL_StatusTypeDef sensor_init(void)
{
    /* 清空变化位图, 丢弃上电抖动可能产生的伪事件 */
    s_changed_flags = 0;
    s_notify_task   = NULL;

    return HAL_OK;
}

/**
 * @brief  读取单个传感器的逻辑触发状态
 */
uint8_t sensor_read(SensorId sensor_id)
{
    GPIO_PinState level;

    if (sensor_id >= SENSOR_ID_COUNT)
    {
        return 0;
    }

    level = HAL_GPIO_ReadPin(s_sensor_hw[sensor_id].port,
                             s_sensor_hw[sensor_id].pin);

    /* 与配置的有效电平比较, 把"电平"翻译成"逻辑状态" */
    return (level == s_sensor_hw[sensor_id].active_level) ? 1U : 0U;
}

/**
 * @brief  一次性读取全部传感器, 打包成位图
 */
uint32_t sensor_read_all(void)
{
    uint32_t bitmap = 0;
    uint8_t  i;

    for (i = 0; i < (uint8_t)SENSOR_ID_COUNT; i++)
    {
        if (sensor_read((SensorId)i) != 0U)
        {
            bitmap |= SENSOR_FLAG(i);
        }
    }

    return bitmap;
}

/**
 * @brief  急停是否按下
 */
uint8_t sensor_emergency_active(void)
{
    return sensor_read(SENSOR_EMERGENCY);
}

/**
 * @brief  注册接收传感器变化通知的任务
 */
void sensor_set_notify_task(void *task_handle)
{
    s_notify_task = (TaskHandle_t)task_handle;
}

/**
 * @brief  取出并清空变化位图
 */
uint32_t sensor_take_changed_flags(void)
{
    uint32_t flags;

    /* 关中断保护"读 + 清零"这两步。
     * 如果不保护, 恰好在读完之后、清零之前来了一次 EXTI,
     * 那次中断置的位会被紧接着的清零操作抹掉, 事件就丢了。
     * taskENTER_CRITICAL 只屏蔽到 configMAX_SYSCALL_INTERRUPT_PRIORITY,
     * 而 EXTI9_5 的优先级是 5(受 FreeRTOS 管理), 因此能被正确屏蔽。 */
    taskENTER_CRITICAL();
    flags = s_changed_flags;
    s_changed_flags = 0;
    taskEXIT_CRITICAL();

    return flags;
}

/**
 * @brief  EXTI 中断的实际处理逻辑
 * @param  gpio_pin 触发中断的引脚号
 * @note   在中断上下文执行，只锁存停止/触限、置位变化位图并通知任务。
 */
void sensor_exti_handler(uint16_t gpio_pin)
{
    uint8_t  i;
    uint8_t  matched = 0;
    BaseType_t higher_priority_woken = pdFALSE;

    stepper_sensor_irq();

    /* 遍历硬件表, 找出是哪个传感器的引脚触发的。
     * PC5~PC8 共用一个中断向量, 所以这一步查表是必需的。 */
    for (i = 0; i < (uint8_t)SENSOR_ID_COUNT; i++)
    {
        if (s_sensor_hw[i].pin == gpio_pin)
        {
            s_changed_flags |= SENSOR_FLAG(i);
            matched = 1;
            /* 不 break: 理论上允许多个传感器共用同一引脚号(不同端口),
             * 全部置位更安全 */
        }
    }

    if (matched == 0U)
    {
        return;     /* 不是本模块关心的引脚, 直接忽略 */
    }

    /* 通知等待的任务(通常是 SafetyTask)去处理。
     * vTaskNotifyGiveFromISR 是中断安全版本, 相当于给任务"发一个信号量"。 */
    if (s_notify_task != NULL)
    {
        vTaskNotifyGiveFromISR(s_notify_task, &higher_priority_woken);

        /* 如果被唤醒的任务优先级比当前任务高, 请求退出中断时立即切换,
         * 这样 SafetyTask 能在中断返回后马上跑起来。 */
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

/**
 * @brief  HAL 的 EXTI 回调, 由 HAL_GPIO_EXTI_IRQHandler() 调用
 * @param  GPIO_Pin 触发中断的引脚号
 * @note   HAL 库里这是 __weak 函数, 在这里重新实现即可覆盖。
 *         整个工程只能有一处实现, 因此统一放在 bsp_sensors.c。
 *         以后若有别的模块也需要 EXTI, 在这里分发给对应模块的处理函数。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    sensor_exti_handler(GPIO_Pin);
}
