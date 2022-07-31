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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define MB_TX_Pin GPIO_PIN_2
#define MB_TX_GPIO_Port GPIOA
#define MB_RX_Pin GPIO_PIN_3
#define MB_RX_GPIO_Port GPIOA
#define RF_NSS_Pin GPIO_PIN_4
#define RF_NSS_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_5
#define RF_SCK_GPIO_Port GPIOA
#define RF_MISO_Pin GPIO_PIN_6
#define RF_MISO_GPIO_Port GPIOA
#define RF_MOSI_Pin GPIO_PIN_7
#define RF_MOSI_GPIO_Port GPIOA
#define RF_RST_Pin GPIO_PIN_0
#define RF_RST_GPIO_Port GPIOB
#define RF_GPIO0_Pin GPIO_PIN_1
#define RF_GPIO0_GPIO_Port GPIOB
#define RF_GPIO1_Pin GPIO_PIN_2
#define RF_GPIO1_GPIO_Port GPIOB
#define RF_IRQ_Pin GPIO_PIN_10
#define RF_IRQ_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define LED0D0_Pin GPIO_PIN_0
#define LED0D0_GPIO_Port GPIOD
#define LED1D1_Pin GPIO_PIN_1
#define LED1D1_GPIO_Port GPIOD
#define LED2D2_Pin GPIO_PIN_2
#define LED2D2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOD
#define GSM_PWR_Pin GPIO_PIN_3
#define GSM_PWR_GPIO_Port GPIOB
#define GSM_INT_Pin GPIO_PIN_4
#define GSM_INT_GPIO_Port GPIOB
#define GSM_RST_Pin GPIO_PIN_5
#define GSM_RST_GPIO_Port GPIOB
#define GSM_RTS_Pin GPIO_PIN_6
#define GSM_RTS_GPIO_Port GPIOB
#define GSM_CTS_Pin GPIO_PIN_7
#define GSM_CTS_GPIO_Port GPIOB
#define GSM_TX_Pin GPIO_PIN_8
#define GSM_TX_GPIO_Port GPIOB
#define GSM_RX_Pin GPIO_PIN_9
#define GSM_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
