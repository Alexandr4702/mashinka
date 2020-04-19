/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED1_Pin GPIO_PIN_13
#define USER_LED1_GPIO_Port GPIOC
#define MOTOR2_2_Pin GPIO_PIN_4
#define MOTOR2_2_GPIO_Port GPIOA
#define MOTOR2_1_Pin GPIO_PIN_5
#define MOTOR2_1_GPIO_Port GPIOA
#define MOTOR1_2_Pin GPIO_PIN_6
#define MOTOR1_2_GPIO_Port GPIOA
#define MOTOR1_1_Pin GPIO_PIN_7
#define MOTOR1_1_GPIO_Port GPIOA
#define MOTOR4_1_Pin GPIO_PIN_4
#define MOTOR4_1_GPIO_Port GPIOC
#define MOTOR4_2_Pin GPIO_PIN_5
#define MOTOR4_2_GPIO_Port GPIOC
#define MOTOR3_1_Pin GPIO_PIN_0
#define MOTOR3_1_GPIO_Port GPIOB
#define MOTOR3_2_Pin GPIO_PIN_1
#define MOTOR3_2_GPIO_Port GPIOB
#define ALL_EN_PIN_ERROR_Pin GPIO_PIN_2
#define ALL_EN_PIN_ERROR_GPIO_Port GPIOB
#define USER_LED3_Pin GPIO_PIN_9
#define USER_LED3_GPIO_Port GPIOA
#define USER_LED2_Pin GPIO_PIN_10
#define USER_LED2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
