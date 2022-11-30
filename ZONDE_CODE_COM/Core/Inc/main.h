/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
void calculate_calibration(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_OSC32_Pin GPIO_PIN_14
#define RTC_OSC32_GPIO_Port GPIOC
#define MB_TX_Pin GPIO_PIN_0
#define MB_TX_GPIO_Port GPIOA
#define MB_RX_Pin GPIO_PIN_1
#define MB_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_4
#define LED_3_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define RF_NSS_Pin GPIO_PIN_0
#define RF_NSS_GPIO_Port GPIOB
#define RF_IRQ_Pin GPIO_PIN_1
#define RF_IRQ_GPIO_Port GPIOB
#define RF_DIO0_Pin GPIO_PIN_8
#define RF_DIO0_GPIO_Port GPIOA
#define RF_RST_Pin GPIO_PIN_6
#define RF_RST_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_12
#define LED_0_GPIO_Port GPIOA
#define GSM_RST_Pin GPIO_PIN_15
#define GSM_RST_GPIO_Port GPIOA
#define GSM_GPIO1_INT_Pin GPIO_PIN_3
#define GSM_GPIO1_INT_GPIO_Port GPIOB
#define GSM_PWR_Pin GPIO_PIN_4
#define GSM_PWR_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define GSM_TX_Pin GPIO_PIN_6
#define GSM_TX_GPIO_Port GPIOB
#define GSM_RX_Pin GPIO_PIN_7
#define GSM_RX_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_8
#define DEBUG_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
