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
#define LED_PIN_Pin GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define DIG_OUT_1_Pin GPIO_PIN_2
#define DIG_OUT_1_GPIO_Port GPIOA
#define DIG_OUT_2_Pin GPIO_PIN_3
#define DIG_OUT_2_GPIO_Port GPIOA
#define DIG_OUT_3_Pin GPIO_PIN_4
#define DIG_OUT_3_GPIO_Port GPIOA
#define EVSE_PILOT_Pin GPIO_PIN_6
#define EVSE_PILOT_GPIO_Port GPIOA
#define EVSE_PILOT_EXTI_IRQn EXTI9_5_IRQn
#define CHARGER3_ENABLE_Pin GPIO_PIN_12
#define CHARGER3_ENABLE_GPIO_Port GPIOB
#define CHARGER2_ENABLE_Pin GPIO_PIN_13
#define CHARGER2_ENABLE_GPIO_Port GPIOB
#define CHARGER1_ENABLE_Pin GPIO_PIN_14
#define CHARGER1_ENABLE_GPIO_Port GPIOB
#define EVSE_ACTIVATE_Pin GPIO_PIN_15
#define EVSE_ACTIVATE_GPIO_Port GPIOB
#define DIG_IN_1_Pin GPIO_PIN_3
#define DIG_IN_1_GPIO_Port GPIOB
#define DIG_IN_2_Pin GPIO_PIN_4
#define DIG_IN_2_GPIO_Port GPIOB
#define CHARGER1_ACTIVATE_Pin GPIO_PIN_5
#define CHARGER1_ACTIVATE_GPIO_Port GPIOB
#define CHARGER2_ACTIVATE_Pin GPIO_PIN_6
#define CHARGER2_ACTIVATE_GPIO_Port GPIOB
#define CHARGER3_ACTIVATE_Pin GPIO_PIN_7
#define CHARGER3_ACTIVATE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
