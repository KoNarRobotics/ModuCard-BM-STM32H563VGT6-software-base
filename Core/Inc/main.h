/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
void MX_USB_PCD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_1_Pin GPIO_PIN_3
#define USER_LED_1_GPIO_Port GPIOE
#define USER_LED_2_Pin GPIO_PIN_4
#define USER_LED_2_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOE
#define USR_BUTTON_Pin GPIO_PIN_6
#define USR_BUTTON_GPIO_Port GPIOE
#define IMU_EXT_NRESET_Pin GPIO_PIN_8
#define IMU_EXT_NRESET_GPIO_Port GPIOE
#define IMU_FSYNC_Pin GPIO_PIN_11
#define IMU_FSYNC_GPIO_Port GPIOE
#define IMU_INT_Pin GPIO_PIN_12
#define IMU_INT_GPIO_Port GPIOE
#define BOOT_EN_Pin GPIO_PIN_7
#define BOOT_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
