/**
 * @file    bsp_buzzer.c
 * @brief   蜂鸣器驱动实现 (PA1 = TIM2_CH2 PWM)
 * @note    实现要点:
 *          1. 不重复初始化 TIM2 —— CubeMX 生成的 MX_TIM2_Init() 已经做完了
 *             基础配置和 PA1 的复用引脚配置, 这里只做"启停 PWM"。
 *          2. 发声 = HAL_TIM_PWM_Start(), 静音 = HAL_TIM_PWM_Stop()。
 *             用启停代替"占空比设 0", 好处是停止后引脚保持低电平,
 *             无源蜂鸣器不会有残留电流。
 *          3. 改频率时同步更新 Pulse = ARR/2, 保证占空比恒为 50%,
 *             50% 方波驱动无源蜂鸣器的能量利用率最高。
 */

#include "bsp_buzzer.h"
#include "tim.h"

/* TIM2 句柄由 CubeMX 在 tim.c 中定义 */
extern TIM_HandleTypeDef htim2;

/* 蜂鸣器使用的 PWM 通道: PA1 对应 TIM2 通道 2 */
#define BUZZER_TIM_CHANNEL      TIM_CHANNEL_2

/* 定时器计数时钟: 100MHz / (Prescaler 99 + 1) = 1MHz
 * 即计数器每递增 1 耗时 1us, 便于用频率直接算 ARR */
#define BUZZER_TIM_CLK_HZ       1000000UL

/* 频率保护范围: 超出这个范围的无源蜂鸣器基本不出声, 视为参数错误 */
#define BUZZER_FREQ_MIN_HZ      500UL
#define BUZZER_FREQ_MAX_HZ      10000UL

/* 驱动内部状态标志 */
static uint8_t s_buzzer_inited = 0;     /* 是否已初始化 */
static uint8_t s_buzzer_playing = 0;    /* 当前是否正在发声 */

/**
 * @brief  蜂鸣器初始化
 * @note   只做状态复位并确保静音, 不重复配置定时器。
 */
HAL_StatusTypeDef buzzer_init(void)
{
    /* 先确保停在静音状态。
     * 这里忽略 Stop 的返回值: 如果 PWM 本来就没启动, HAL 会返回错误,
     * 但那正是我们想要的结果(静音), 不算失败。 */
    (void)HAL_TIM_PWM_Stop(&htim2, BUZZER_TIM_CHANNEL);

    s_buzzer_playing = 0;
    s_buzzer_inited = 1;

    return HAL_OK;
}

/**
 * @brief  开始持续发声
 */
HAL_StatusTypeDef buzzer_on(void)
{
    if (s_buzzer_inited == 0)
    {
        return HAL_ERROR;       /* 未初始化, 拒绝操作 */
    }

    if (s_buzzer_playing != 0)
    {
        return HAL_OK;          /* 已经在响, 幂等处理直接返回 */
    }

    if (HAL_TIM_PWM_Start(&htim2, BUZZER_TIM_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_buzzer_playing = 1;
    return HAL_OK;
}

/**
 * @brief  停止发声
 */
HAL_StatusTypeDef buzzer_off(void)
{
    if (s_buzzer_inited == 0)
    {
        return HAL_ERROR;
    }

    /* 无条件停止: 即使标志显示没在响也执行一次, 避免状态不一致时漏关 */
    (void)HAL_TIM_PWM_Stop(&htim2, BUZZER_TIM_CHANNEL);

    s_buzzer_playing = 0;
    return HAL_OK;
}

/**
 * @brief  设置发声频率, 占空比固定 50%
 */
HAL_StatusTypeDef buzzer_set_freq(uint32_t freq_hz)
{
    uint32_t arr;

    if ((freq_hz < BUZZER_FREQ_MIN_HZ) || (freq_hz > BUZZER_FREQ_MAX_HZ))
    {
        return HAL_ERROR;
    }

    /* ARR = 计数时钟 / 目标频率 - 1
     * 例: 1MHz / 3000Hz - 1 = 332, 与 CubeMX 的 333 基本一致 */
    arr = (BUZZER_TIM_CLK_HZ / freq_hz) - 1UL;

    /* 直接写寄存器修改周期和比较值。
     * __HAL_TIM_SET_AUTORELOAD / SET_COMPARE 是 HAL 提供的宏,
     * 可以在 PWM 运行中动态修改, 无需停止定时器。 */
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, BUZZER_TIM_CHANNEL, arr / 2UL);

    /* 如果当前计数值已经超过新的 ARR, 计数器要等溢出一整圈才恢复正常,
     * 手动清零可以让新频率立即生效。 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    return HAL_OK;
}

/**
 * @brief  内部辅助: 响指定毫秒后自动停止(阻塞)
 * @param  ms 发声时长
 */
static HAL_StatusTypeDef buzzer_beep_ms(uint32_t ms)
{
    if (buzzer_on() != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(ms);              /* HAL 时基为 TIM11, 与 FreeRTOS 调度器无关 */

    return buzzer_off();
}

/**
 * @brief  短提示音 50ms
 */
HAL_StatusTypeDef buzzer_beep_short(void)
{
    return buzzer_beep_ms(BUZZER_MS_SHORT);
}

/**
 * @brief  完成音 200ms
 */
HAL_StatusTypeDef buzzer_beep_done(void)
{
    return buzzer_beep_ms(BUZZER_MS_DONE);
}

/**
 * @brief  错误音: 三声短促急鸣
 * @note   "响 60ms - 停 40ms" 重复三次, 听感上比单声长鸣更有警示性。
 */
HAL_StatusTypeDef buzzer_beep_error(void)
{
    uint8_t i;

    for (i = 0; i < 3U; i++)
    {
        if (buzzer_beep_ms(60U) != HAL_OK)
        {
            return HAL_ERROR;
        }

        if (i < 2U)             /* 最后一声后面不用再停顿 */
        {
            HAL_Delay(40U);
        }
    }

    return HAL_OK;
}

/**
 * @brief  自检
 */
HAL_StatusTypeDef buzzer_selftest(void)
{
    return buzzer_beep_short();
}
