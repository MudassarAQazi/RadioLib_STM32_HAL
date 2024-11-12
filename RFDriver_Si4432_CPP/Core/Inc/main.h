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
#include "stm32l0xx_hal.h"

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
#define debugUart huart2
#define RF_SPI hspi1
#define RF_SDN_Si4432_Pin GPIO_PIN_1
#define RF_SDN_Si4432_GPIO_Port GPIOA
#define RF_nIRQ_Si4432_Pin GPIO_PIN_3
#define RF_nIRQ_Si4432_GPIO_Port GPIOA
#define RF_nIRQ_Si4432_EXTI_IRQn EXTI2_3_IRQn
#define RF_nSEL_Si4432_Pin GPIO_PIN_4
#define RF_nSEL_Si4432_GPIO_Port GPIOA
#define RF_SCK_Si4432_Pin GPIO_PIN_5
#define RF_SCK_Si4432_GPIO_Port GPIOA
#define RF_MISO_SDO_Si4432_Pin GPIO_PIN_6
#define RF_MISO_SDO_Si4432_GPIO_Port GPIOA
#define RF_MOSI_SDI_Si4432_Pin GPIO_PIN_7
#define RF_MOSI_SDI_Si4432_GPIO_Port GPIOA
#define heartBeatLed_Pin GPIO_PIN_1
#define heartBeatLed_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
