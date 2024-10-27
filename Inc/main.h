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
#define RPI_extra_Pin GPIO_PIN_0
#define RPI_extra_GPIO_Port GPIOA
#define biasfb0_Pin GPIO_PIN_4
#define biasfb0_GPIO_Port GPIOA
#define biasfb1_Pin GPIO_PIN_5
#define biasfb1_GPIO_Port GPIOA
#define inj_led_Pin GPIO_PIN_6
#define inj_led_GPIO_Port GPIOA
#define pwr_led_Pin GPIO_PIN_7
#define pwr_led_GPIO_Port GPIOA
#define flag_Pin GPIO_PIN_1
#define flag_GPIO_Port GPIOB
#define evt_led_Pin GPIO_PIN_8
#define evt_led_GPIO_Port GPIOA
#define GPSPPS_Pin GPIO_PIN_15
#define GPSPPS_GPIO_Port GPIOA
#define ch_b_or_Pin GPIO_PIN_11
#define ch_b_or_GPIO_Port GPIOC
#define ch_a_or_Pin GPIO_PIN_12
#define ch_a_or_GPIO_Port GPIOC
#define trig_out_Pin GPIO_PIN_3
#define trig_out_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
