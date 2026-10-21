/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC
#define MOTOR_A_IN1_Pin GPIO_PIN_0
#define MOTOR_A_IN1_GPIO_Port GPIOC
#define MOTOR_A_IN2_Pin GPIO_PIN_1
#define MOTOR_A_IN2_GPIO_Port GPIOC
#define MOTOR_A_IN3_Pin GPIO_PIN_2
#define MOTOR_A_IN3_GPIO_Port GPIOC
#define MOTOR_A_IN4_Pin GPIO_PIN_3
#define MOTOR_A_IN4_GPIO_Port GPIOC
#define KEY_USER_Pin GPIO_PIN_0
#define KEY_USER_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_1
#define BUZZER_PWM_GPIO_Port GPIOA
#define RC522_CS_Pin GPIO_PIN_4
#define RC522_CS_GPIO_Port GPIOA
#define RC522_RST_Pin GPIO_PIN_4
#define RC522_RST_GPIO_Port GPIOC
#define IR_SENSOR_1_Pin GPIO_PIN_5
#define IR_SENSOR_1_GPIO_Port GPIOC
#define IR_SENSOR_1_EXTI_IRQn EXTI9_5_IRQn
#define MOTOR_B_IN1_Pin GPIO_PIN_0
#define MOTOR_B_IN1_GPIO_Port GPIOB
#define MOTOR_B_IN2_Pin GPIO_PIN_1
#define MOTOR_B_IN2_GPIO_Port GPIOB
#define EMERGENCY_STOP_Pin GPIO_PIN_2
#define EMERGENCY_STOP_GPIO_Port GPIOB
#define EMERGENCY_STOP_EXTI_IRQn EXTI2_IRQn
#define MOTOR_B_IN4_Pin GPIO_PIN_10
#define MOTOR_B_IN4_GPIO_Port GPIOB
#define IR_SENSOR_2_Pin GPIO_PIN_6
#define IR_SENSOR_2_GPIO_Port GPIOC
#define IR_SENSOR_2_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_SW_1_Pin GPIO_PIN_7
#define LIMIT_SW_1_GPIO_Port GPIOC
#define LIMIT_SW_1_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_SW_2_Pin GPIO_PIN_8
#define LIMIT_SW_2_GPIO_Port GPIOC
#define LIMIT_SW_2_EXTI_IRQn EXTI9_5_IRQn
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOA
#define MOTOR_B_IN3_Pin GPIO_PIN_4
#define MOTOR_B_IN3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
