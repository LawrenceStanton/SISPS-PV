/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.hpp
  * @brief          : Header for main.hpp.
  * @author			: Lawrence Stanton
  ******************************************************************************
  * @attention
  *
  * © LD Stanton 2022
  * 
  * This file and its content are the copyright property of the author. All 
  * rights are reserved. No warranty is given. No liability is assumed.
  * Confidential unless licensed otherwise. If licensed, refer to the 
  * accompanying file "LICENCE" for license details.
  * 
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
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
#include "stm32g0xx_hal.h"

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
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOA
#define FUSE_OC_Pin GPIO_PIN_10
#define FUSE_OC_GPIO_Port GPIOB
#define FUSE_OK_Pin GPIO_PIN_11
#define FUSE_OK_GPIO_Port GPIOB
#define PGOOD_Pin GPIO_PIN_12
#define PGOOD_GPIO_Port GPIOB
#define PM_FORCE_Pin GPIO_PIN_8
#define PM_FORCE_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_11
#define STOP_GPIO_Port GPIOA
#define EEPROM_WC_Pin GPIO_PIN_12
#define EEPROM_WC_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
