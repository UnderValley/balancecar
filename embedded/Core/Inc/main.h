/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define PWM_OUTPUT_B_Pin GPIO_PIN_0
#define PWM_OUTPUT_B_GPIO_Port GPIOA
#define PWM_OUTPUT_A_Pin GPIO_PIN_1
#define PWM_OUTPUT_A_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define ENCODER_INPUT_A1_Pin GPIO_PIN_6
#define ENCODER_INPUT_A1_GPIO_Port GPIOA
#define ENCODER_INPUT_A2_Pin GPIO_PIN_7
#define ENCODER_INPUT_A2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_10
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOB
#define OLED_DO_Pin GPIO_PIN_12
#define OLED_DO_GPIO_Port GPIOB
#define OLED_DI_Pin GPIO_PIN_13
#define OLED_DI_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_14
#define OLED_RES_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_15
#define OLED_DC_GPIO_Port GPIOB
#define ENCODER_INPUT_B1_Pin GPIO_PIN_8
#define ENCODER_INPUT_B1_GPIO_Port GPIOA
#define ENCODER_INPUT_B2_Pin GPIO_PIN_9
#define ENCODER_INPUT_B2_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_10
#define BIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_11
#define BIN1_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_12
#define STBY_GPIO_Port GPIOA
#define PS2_DI_Pin GPIO_PIN_15
#define PS2_DI_GPIO_Port GPIOA
#define PS2_DO_CMD_Pin GPIO_PIN_3
#define PS2_DO_CMD_GPIO_Port GPIOB
#define PS2_CS_SEL_Pin GPIO_PIN_4
#define PS2_CS_SEL_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_5
#define PS2_CLK_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB
#define MPU6050_INT_Pin GPIO_PIN_8
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
