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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI160.h"
#include "HuskyLens.h"
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
#define i3_Pin GPIO_PIN_0
#define i3_GPIO_Port GPIOC
#define i3_EXTI_IRQn EXTI0_IRQn
#define l2_Pin GPIO_PIN_1
#define l2_GPIO_Port GPIOC
#define l2_EXTI_IRQn EXTI1_IRQn
#define k1_Pin GPIO_PIN_2
#define k1_GPIO_Port GPIOC
#define k1_EXTI_IRQn EXTI2_IRQn
#define j0_Pin GPIO_PIN_3
#define j0_GPIO_Port GPIOC
#define j0_EXTI_IRQn EXTI3_IRQn
#define x_Pin GPIO_PIN_0
#define x_GPIO_Port GPIOA
#define y_Pin GPIO_PIN_1
#define y_GPIO_Port GPIOA
#define led_key_Pin GPIO_PIN_0
#define led_key_GPIO_Port GPIOB
#define MOTAR_Pin GPIO_PIN_6
#define MOTAR_GPIO_Port GPIOC
#define start_Pin GPIO_PIN_4
#define start_GPIO_Port GPIOB
#define start_EXTI_IRQn EXTI4_IRQn
#define pause_Pin GPIO_PIN_5
#define pause_GPIO_Port GPIOB
#define pause_EXTI_IRQn EXTI9_5_IRQn
#define full_screem_Pin GPIO_PIN_6
#define full_screem_GPIO_Port GPIOB
#define full_screem_EXTI_IRQn EXTI9_5_IRQn
#define space_Pin GPIO_PIN_7
#define space_GPIO_Port GPIOB
#define space_EXTI_IRQn EXTI9_5_IRQn
#define p5_Pin GPIO_PIN_8
#define p5_GPIO_Port GPIOB
#define p5_EXTI_IRQn EXTI9_5_IRQn
#define o4_Pin GPIO_PIN_9
#define o4_GPIO_Port GPIOB
#define o4_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
