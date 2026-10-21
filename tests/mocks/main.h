#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { unsigned id; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
typedef struct { uint32_t DEMCR; } TestDebug;
extern GPIO_TypeDef test_ports[3];
extern TestDebug test_debug;
extern uint32_t SystemCoreClock;
TestDwt *test_dwt(void);
#define DWT (test_dwt())
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define MOTOR_A_IN1_GPIO_Port (&test_ports[0])
#define MOTOR_A_IN2_GPIO_Port (&test_ports[0])
#define MOTOR_A_IN3_GPIO_Port (&test_ports[0])
#define MOTOR_A_IN4_GPIO_Port (&test_ports[0])
#define MOTOR_B_IN1_GPIO_Port (&test_ports[1])
#define MOTOR_B_IN2_GPIO_Port (&test_ports[1])
#define MOTOR_B_IN3_GPIO_Port (&test_ports[1])
#define MOTOR_B_IN4_GPIO_Port (&test_ports[1])
#define MOTOR_A_IN1_Pin 1U
#define MOTOR_A_IN2_Pin 2U
#define MOTOR_A_IN3_Pin 4U
#define MOTOR_A_IN4_Pin 8U
#define MOTOR_B_IN1_Pin 1U
#define MOTOR_B_IN2_Pin 2U
#define MOTOR_B_IN3_Pin 4U
#define MOTOR_B_IN4_Pin 8U
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
#endif
