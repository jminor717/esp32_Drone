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
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SERVO_PID_PERIOD 10
#define SERVO_PID_OFFSET 2
#define SERVO_NUM_DIVISIONS 5 //SERVO_PID_PERIOD / SERVO_PID_OFFSET
#define HUMAN_READABLE_SPI 100


extern bool SPI_REFRESH;
extern bool DSHOT_MOTOR_REFRESH;
extern bool SERVO_REFRESH[SERVO_NUM_DIVISIONS];
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void CustomTickHandler(uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define Dshot1_Pin GPIO_PIN_0
#define Dshot1_GPIO_Port GPIOA
#define Dshot2_Pin GPIO_PIN_1
#define Dshot2_GPIO_Port GPIOA
#define Dshot3_Pin GPIO_PIN_2
#define Dshot3_GPIO_Port GPIOA
#define Dshot4_Pin GPIO_PIN_3
#define Dshot4_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOA
#define AIN3_Pin GPIO_PIN_6
#define AIN3_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOA
#define M3P_Pin GPIO_PIN_0
#define M3P_GPIO_Port GPIOB
#define ESP32_SPI_MODE_Pin GPIO_PIN_15
#define ESP32_SPI_MODE_GPIO_Port GPIOB
#define ESP32_SPI_MODE_EXTI_IRQn EXTI15_10_IRQn
#define M2N_Pin GPIO_PIN_8
#define M2N_GPIO_Port GPIOA
#define M1P_Pin GPIO_PIN_9
#define M1P_GPIO_Port GPIOA
#define M1N_Pin GPIO_PIN_10
#define M1N_GPIO_Port GPIOA
#define M2P_Pin GPIO_PIN_15
#define M2P_GPIO_Port GPIOA
#define SPI1_CLK_Pin GPIO_PIN_3
#define SPI1_CLK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define M3N_Pin GPIO_PIN_6
#define M3N_GPIO_Port GPIOB
#define M4N_Pin GPIO_PIN_7
#define M4N_GPIO_Port GPIOB
#define M4P_Pin GPIO_PIN_8
#define M4P_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SPI1_CLK_Pin_NUM 3
#define SPI1_MISO_Pin_NUM 4
#define SPI1_MOSI_Pin_NUM 5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
