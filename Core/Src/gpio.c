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
     PA1   ------> OCTOSPIM_P1_IO3
     PA2   ------> S_TIM2_CH3
     PA3   ------> OCTOSPIM_P1_IO2
     PC4   ------> ADCx_INP4
     PB0   ------> OCTOSPIM_P1_IO1
     PE9   ------> S_TIM1_CH1
     PE13   ------> S_TIM1_CH3
     PD11   ------> OCTOSPIM_P1_IO0
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC24_1__OUTPUT_Pin|DC24_0__OUTPUT_Pin|DC5__OUTPUT_Pin|BMI088_ACCEL_CS_Pin
                          |BMI088_GYRO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DC24_1__OUTPUT_Pin DC24_0__OUTPUT_Pin DC5__OUTPUT_Pin */
  GPIO_InitStruct.Pin = DC24_1__OUTPUT_Pin|DC24_0__OUTPUT_Pin|DC5__OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI088_ACCEL_CS_Pin BMI088_GYRO_CS_Pin */
  GPIO_InitStruct.Pin = BMI088_ACCEL_CS_Pin|BMI088_GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q64_HOLD__OSPI_IO3_Pin */
  GPIO_InitStruct.Pin = W25Q64_HOLD__OSPI_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(W25Q64_HOLD__OSPI_IO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SERVO_2__PWM_Pin */
  GPIO_InitStruct.Pin = SERVO_2__PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(SERVO_2__PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q64_WP__OSPI_IO2_Pin */
  GPIO_InitStruct.Pin = W25Q64_WP__OSPI_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_OCTOSPIM_P1;
  HAL_GPIO_Init(W25Q64_WP__OSPI_IO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_VOLTAGE__ADC_Pin */
  GPIO_InitStruct.Pin = BAT_VOLTAGE__ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_VOLTAGE__ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q64_SO__OSPI_IO1_Pin */
  GPIO_InitStruct.Pin = W25Q64_SO__OSPI_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_OCTOSPIM_P1;
  HAL_GPIO_Init(W25Q64_SO__OSPI_IO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO_1__PWM_Pin SERVO_0__PWM_Pin */
  GPIO_InitStruct.Pin = SERVO_1__PWM_Pin|SERVO_0__PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI088_ACCEL_INTERRUPT_Pin BMI088_GYRO__INTERRUPT_Pin */
  GPIO_InitStruct.Pin = BMI088_ACCEL_INTERRUPT_Pin|BMI088_GYRO__INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q64_SI__OSPI_IO0_Pin */
  GPIO_InitStruct.Pin = W25Q64_SI__OSPI_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(W25Q64_SI__OSPI_IO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY__INPUT_Pin */
  GPIO_InitStruct.Pin = KEY__INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY__INPUT_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
