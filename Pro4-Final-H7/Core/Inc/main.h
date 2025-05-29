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
#include "stm32h7xx_hal.h"

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
#define SPI4_CS_M1_Pin GPIO_PIN_13
#define SPI4_CS_M1_GPIO_Port GPIOC
#define SPI4_CS_M2_Pin GPIO_PIN_14
#define SPI4_CS_M2_GPIO_Port GPIOC
#define SPI4_CS_M3_Pin GPIO_PIN_15
#define SPI4_CS_M3_GPIO_Port GPIOC
#define SPI2_CS_ADC1_Pin GPIO_PIN_0
#define SPI2_CS_ADC1_GPIO_Port GPIOH
#define SPI2_CS_ADC2_Pin GPIO_PIN_1
#define SPI2_CS_ADC2_GPIO_Port GPIOH
#define SPI2_CS_ADC3_Pin GPIO_PIN_0
#define SPI2_CS_ADC3_GPIO_Port GPIOC
#define M1_EN_DIAG_Pin GPIO_PIN_3
#define M1_EN_DIAG_GPIO_Port GPIOC
#define DRIVER_M1_EN1_Pin GPIO_PIN_3
#define DRIVER_M1_EN1_GPIO_Port GPIOA
#define DRIVER_M1_EN2_Pin GPIO_PIN_4
#define DRIVER_M1_EN2_GPIO_Port GPIOA
#define DRIVER_M1_EN3_Pin GPIO_PIN_5
#define DRIVER_M1_EN3_GPIO_Port GPIOA
#define DRIVER_M2_EN1_Pin GPIO_PIN_4
#define DRIVER_M2_EN1_GPIO_Port GPIOC
#define DRIVER_M2_EN2_Pin GPIO_PIN_5
#define DRIVER_M2_EN2_GPIO_Port GPIOC
#define DRIVER_M2_EN3_Pin GPIO_PIN_1
#define DRIVER_M2_EN3_GPIO_Port GPIOB
#define M2_EN_DIAG_Pin GPIO_PIN_2
#define M2_EN_DIAG_GPIO_Port GPIOB
#define DRIVER_M3_EN1_Pin GPIO_PIN_10
#define DRIVER_M3_EN1_GPIO_Port GPIOE
#define DRIVER_M3_EN2_Pin GPIO_PIN_12
#define DRIVER_M3_EN2_GPIO_Port GPIOE
#define DRIVER_M3_EN3_Pin GPIO_PIN_14
#define DRIVER_M3_EN3_GPIO_Port GPIOE
#define M3_EN_DIAG_Pin GPIO_PIN_15
#define M3_EN_DIAG_GPIO_Port GPIOE
#define LED_Error3_Pin GPIO_PIN_8
#define LED_Error3_GPIO_Port GPIOD
#define LED_Error2_Pin GPIO_PIN_9
#define LED_Error2_GPIO_Port GPIOD
#define LED_Error1_Pin GPIO_PIN_10
#define LED_Error1_GPIO_Port GPIOD
#define LED_SYSTEMSTATUS_Pin GPIO_PIN_11
#define LED_SYSTEMSTATUS_GPIO_Port GPIOD
#define LED_COM_FLUGREGLER_Pin GPIO_PIN_12
#define LED_COM_FLUGREGLER_GPIO_Port GPIOD
#define USART6_TX_FLUGREGLER_Pin GPIO_PIN_6
#define USART6_TX_FLUGREGLER_GPIO_Port GPIOC
#define USART6_RX_FLUGREGLER_Pin GPIO_PIN_7
#define USART6_RX_FLUGREGLER_GPIO_Port GPIOC
#define INT_MAG_Pin GPIO_PIN_9
#define INT_MAG_GPIO_Port GPIOA
#define USART3_TX_VCOM_Pin GPIO_PIN_10
#define USART3_TX_VCOM_GPIO_Port GPIOC
#define USART3_RX_VCOM_Pin GPIO_PIN_11
#define USART3_RX_VCOM_GPIO_Port GPIOC
#define INT4_GYRO_Pin GPIO_PIN_1
#define INT4_GYRO_GPIO_Port GPIOD
#define INT3_GYRO_Pin GPIO_PIN_2
#define INT3_GYRO_GPIO_Port GPIOD
#define INT3_GYRO_EXTI_IRQn EXTI2_IRQn
#define INT2_ACC_Pin GPIO_PIN_3
#define INT2_ACC_GPIO_Port GPIOD
#define INT2_ACC_EXTI_IRQn EXTI3_IRQn
#define INT1_ACC_Pin GPIO_PIN_4
#define INT1_ACC_GPIO_Port GPIOD
#define INT1_ACC_EXTI_IRQn EXTI4_IRQn
#define SPI1_CS_GYRO_Pin GPIO_PIN_5
#define SPI1_CS_GYRO_GPIO_Port GPIOD
#define SPI1_CS_ACC_Pin GPIO_PIN_6
#define SPI1_CS_ACC_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
