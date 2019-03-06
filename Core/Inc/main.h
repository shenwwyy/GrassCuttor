/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOC
#define Battery1_Pin GPIO_PIN_0
#define Battery1_GPIO_Port GPIOC
#define Battery2_Pin GPIO_PIN_1
#define Battery2_GPIO_Port GPIOC
#define Battery3_Pin GPIO_PIN_2
#define Battery3_GPIO_Port GPIOC
#define Battery4_Pin GPIO_PIN_3
#define Battery4_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOA
#define MOTOR_EN1_Pin GPIO_PIN_2
#define MOTOR_EN1_GPIO_Port GPIOB
#define MOTOR_EN2_Pin GPIO_PIN_7
#define MOTOR_EN2_GPIO_Port GPIOE
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_9
#define SW2_GPIO_Port GPIOD
#define MOTOR_EN3_Pin GPIO_PIN_10
#define MOTOR_EN3_GPIO_Port GPIOD
#define MOTOR_EN4_Pin GPIO_PIN_11
#define MOTOR_EN4_GPIO_Port GPIOD
#define PWR_EN1_Pin GPIO_PIN_8
#define PWR_EN1_GPIO_Port GPIOB
#define PWR_EN2_Pin GPIO_PIN_9
#define PWR_EN2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_0
#define KEY3_GPIO_Port GPIOE
#define KEY3_EXTI_IRQn EXTI0_IRQn
#define KEY4_Pin GPIO_PIN_1
#define KEY4_GPIO_Port GPIOE
#define KEY4_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */
#if 0
#ifdef __NVIC_PRIO_BITS
#undef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS      4
#endif
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
