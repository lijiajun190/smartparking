#include "main.h"
#include "bsp_watchdog.h"
static IWDG_HandleTypeDef watchdog;
void bsp_watchdog_init(void)
{
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_64;
    watchdog.Init.Reload = 4095U; /* LSI=32kHz时约8.2s，实际随LSI偏差变化 */
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) bsp_runtime_panic();
}
void bsp_watchdog_refresh(void)
{
    if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) bsp_runtime_panic();
}
void bsp_runtime_panic(void)
{
    __disable_irq();
    MOTOR_A_IN1_GPIO_Port->BSRR = (uint32_t)MOTOR_A_IN1_Pin << 16U;
    MOTOR_A_IN2_GPIO_Port->BSRR = (uint32_t)MOTOR_A_IN2_Pin << 16U;
    MOTOR_A_IN3_GPIO_Port->BSRR = (uint32_t)MOTOR_A_IN3_Pin << 16U;
    MOTOR_A_IN4_GPIO_Port->BSRR = (uint32_t)MOTOR_A_IN4_Pin << 16U;
    MOTOR_B_IN1_GPIO_Port->BSRR = (uint32_t)MOTOR_B_IN1_Pin << 16U;
    MOTOR_B_IN2_GPIO_Port->BSRR = (uint32_t)MOTOR_B_IN2_Pin << 16U;
    MOTOR_B_IN3_GPIO_Port->BSRR = (uint32_t)MOTOR_B_IN3_Pin << 16U;
    MOTOR_B_IN4_GPIO_Port->BSRR = (uint32_t)MOTOR_B_IN4_Pin << 16U;
    for (;;) { __NOP(); }
}
