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
#define motor_enable_Pin GPIO_PIN_14
#define motor_enable_GPIO_Port GPIOC
#define motor_HI_Z_Pin GPIO_PIN_15
#define motor_HI_Z_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOC
#define DRV_CS_Pin GPIO_PIN_10
#define DRV_CS_GPIO_Port GPIOB
#define DRV_fault_Pin GPIO_PIN_11
#define DRV_fault_GPIO_Port GPIOB
#define LED1_red_Pin GPIO_PIN_15
#define LED1_red_GPIO_Port GPIOA
#define LED2_green_Pin GPIO_PIN_3
#define LED2_green_GPIO_Port GPIOB
#define LED3_blue_Pin GPIO_PIN_6
#define LED3_blue_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
