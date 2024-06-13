/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define TIM2_COUNTER_PERIOD 212
#define TRILL_3V3_Pin GPIO_PIN_0
#define TRILL_3V3_GPIO_Port GPIOA
#define PSOC_PULLUP_SDA_Pin GPIO_PIN_1
#define PSOC_PULLUP_SDA_GPIO_Port GPIOA
#define ADC_DIG_IN_Pin GPIO_PIN_0
#define ADC_DIG_IN_GPIO_Port GPIOB
#define PSOC_EVENT_Pin GPIO_PIN_3
#define PSOC_EVENT_GPIO_Port GPIOB
#define PSOC_EVENT_EXTI_IRQn EXTI3_IRQn
#define SW0_Pin GPIO_PIN_4
#define SW0_GPIO_Port GPIOB
#define SW_LED_A_Pin GPIO_PIN_5
#define SW_LED_A_GPIO_Port GPIOB
#define SW_LED_B_Pin GPIO_PIN_6
#define SW_LED_B_GPIO_Port GPIOB
#define DEBUG3_Pin GPIO_PIN_8
#define DEBUG3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
