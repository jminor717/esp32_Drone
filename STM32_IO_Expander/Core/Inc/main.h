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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define M1P_Pin GPIO_PIN_1
#define M1P_GPIO_Port GPIOA
#define M1N_Pin GPIO_PIN_2
#define M1N_GPIO_Port GPIOA
#define M2N_Pin GPIO_PIN_3
#define M2N_GPIO_Port GPIOA
#define SPI1_SCK_MASTER_Pin GPIO_PIN_5
#define SPI1_SCK_MASTER_GPIO_Port GPIOA
#define SPI1_MISO_MASTER_Pin GPIO_PIN_6
#define SPI1_MISO_MASTER_GPIO_Port GPIOA
#define SPI1_MOSI_MASTER_Pin GPIO_PIN_7
#define SPI1_MOSI_MASTER_GPIO_Port GPIOA
#define M3P_Pin GPIO_PIN_0
#define M3P_GPIO_Port GPIOB
#define M3N_Pin GPIO_PIN_1
#define M3N_GPIO_Port GPIOB
#define SPI2_SCK_SLAVE_Pin GPIO_PIN_13
#define SPI2_SCK_SLAVE_GPIO_Port GPIOB
#define SPI2_MISO_SLAVE_Pin GPIO_PIN_14
#define SPI2_MISO_SLAVE_GPIO_Port GPIOB
#define SPI2_MOSI_SLAVE_Pin GPIO_PIN_15
#define SPI2_MOSI_SLAVE_GPIO_Port GPIOB
#define Dshot4_Pin GPIO_PIN_8
#define Dshot4_GPIO_Port GPIOA
#define Dshot3_Pin GPIO_PIN_9
#define Dshot3_GPIO_Port GPIOA
#define M2P_Pin GPIO_PIN_15
#define M2P_GPIO_Port GPIOA
#define Dshot2_Pin GPIO_PIN_6
#define Dshot2_GPIO_Port GPIOB
#define M4N_Pin GPIO_PIN_7
#define M4N_GPIO_Port GPIOB
#define Dshot1_Pin GPIO_PIN_8
#define Dshot1_GPIO_Port GPIOB
#define M4P_Pin GPIO_PIN_9
#define M4P_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
