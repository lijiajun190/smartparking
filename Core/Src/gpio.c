/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_STATUS_Pin|RC522_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_A_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_A_IN3_Pin|MOTOR_A_IN4_Pin
                          |OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_B_IN1_Pin|MOTOR_B_IN2_Pin|MOTOR_B_IN4_Pin|MOTOR_B_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_STATUS_Pin MOTOR_A_IN1_Pin MOTOR_A_IN2_Pin MOTOR_A_IN3_Pin
                           MOTOR_A_IN4_Pin RC522_RST_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|MOTOR_A_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_A_IN3_Pin
                          |MOTOR_A_IN4_Pin|RC522_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_USER_Pin */
  GPIO_InitStruct.Pin = KEY_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RC522_CS_Pin */
  GPIO_InitStruct.Pin = RC522_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RC522_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_SENSOR_1_Pin IR_SENSOR_2_Pin LIMIT_SW_1_Pin LIMIT_SW_2_Pin */
  GPIO_InitStruct.Pin = IR_SENSOR_1_Pin|IR_SENSOR_2_Pin|LIMIT_SW_1_Pin|LIMIT_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_B_IN1_Pin MOTOR_B_IN2_Pin MOTOR_B_IN4_Pin MOTOR_B_IN3_Pin */
  GPIO_InitStruct.Pin = MOTOR_B_IN1_Pin|MOTOR_B_IN2_Pin|MOTOR_B_IN4_Pin|MOTOR_B_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EMERGENCY_STOP_Pin */
  GPIO_InitStruct.Pin = EMERGENCY_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EMERGENCY_STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
