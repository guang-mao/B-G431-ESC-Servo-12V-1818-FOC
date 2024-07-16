/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tsk_main.h"
#include "HallSensor.h"
#include "motor.h"
#include "mc_math.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_TIM1_Init(void);
void MX_TIM4_Init(void);
void MX_USART2_UART_Init(void);
void MX_CORDIC_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_UL_Pin GPIO_PIN_13
#define PWM_UL_GPIO_Port GPIOC
#define PWM_WL_Pin GPIO_PIN_15
#define PWM_WL_GPIO_Port GPIOB
#define STATUS_Pin GPIO_PIN_6
#define STATUS_GPIO_Port GPIOC
#define PWM_UH_Pin GPIO_PIN_8
#define PWM_UH_GPIO_Port GPIOA
#define PWM_VH_Pin GPIO_PIN_9
#define PWM_VH_GPIO_Port GPIOA
#define PWM_WH_Pin GPIO_PIN_10
#define PWM_WH_GPIO_Port GPIOA
#define PWM_VL_Pin GPIO_PIN_12
#define PWM_VL_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_11
#define BUTTON_GPIO_Port GPIOC
#define M1_HALL_H1_Pin GPIO_PIN_6
#define M1_HALL_H1_GPIO_Port GPIOB
#define M1_HALL_H2_Pin GPIO_PIN_7
#define M1_HALL_H2_GPIO_Port GPIOB
#define M1_HALL_H3_Pin GPIO_PIN_8
#define M1_HALL_H3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
