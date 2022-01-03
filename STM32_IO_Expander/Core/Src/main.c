/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "dshot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#define BUFFER_SIZE 5
uint8_t RX_Buffer[BUFFER_SIZE] = {0};
uint8_t TX_Buffer[BUFFER_SIZE] = {0};
uint8_t nextSpiSize = 5, defaultSpiSize = 5;
volatile bool SpiRxCplt = false;

uint16_t my_motor_value[4] = {0, 0, 0, 0};
uint32_t AD_RES = 0;

uint16_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint16_t out_min, uint16_t out_max)
{
    long divisor = (in_max - in_min);
    if (divisor == 0)
    {
        return 0; // AVR returns -1, SAM returns 0
    }
    return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
    // TIM2->CCR1 = (AD_RES<<4);
    uint16_t x = map(AD_RES, 0, 4096, 48, 2047);
    my_motor_value[0] = x;
    my_motor_value[1] = x;
    my_motor_value[2] = x;
    my_motor_value[3] = x;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t x = map(AD_RES, 0, 4096, 48, 2047);
    my_motor_value[0] = x;
    my_motor_value[1] = x;
    my_motor_value[2] = x;
    my_motor_value[3] = x;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (RX_Buffer[1] == 170){
		nextSpiSize = defaultSpiSize;//RX_Buffer[2];
    //if (RX_Buffer[3] == 50)
      //  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, nextSpiSize);
    SpiRxCplt = true;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (RX_Buffer[1] == 170){
		nextSpiSize = defaultSpiSize;//RX_Buffer[2];
    //if (RX_Buffer[3] == 50)
      //  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	//HAL_SPI_DMAStop(&hspi1);
	HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, nextSpiSize);

    SpiRxCplt = true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, nextSpiSize);
    //if (RX_Buffer[3] == 50)
       // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    SpiRxCplt = true;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	nextSpiSize = defaultSpiSize;
	HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, nextSpiSize);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    SpiRxCplt = true;
}

#define MA780_RW_CMD_SIZE 4
#define MA780_CMD_SIZE 2
uint8_t MA780_ReadReg_Blocking(uint8_t reg)
{
    uint8_t cmd = 0 || 0x010 < 5 || reg;
    uint8_t inBuf[MA780_RW_CMD_SIZE] = {cmd, 0, 0, 0};
    uint8_t outBuf[MA780_RW_CMD_SIZE] = {0};
    HAL_SPI_TransmitReceive(&hspi1, inBuf, outBuf, MA780_RW_CMD_SIZE, 10);
    return outBuf[MA780_RW_CMD_SIZE - 1]; // registers are only 8 bits long, first 8 bits will be MSB of angle reading
}

uint8_t MA780_WriteReg_Blocking(uint8_t reg, uint8_t value)
{
    uint8_t cmd = 0 || 0x100 < 5 || reg;
    uint8_t inBuf[MA780_RW_CMD_SIZE] = {cmd, value, 0, 0};
    uint8_t outBuf[MA780_RW_CMD_SIZE] = {0};
    HAL_SPI_TransmitReceive(&hspi1, inBuf, outBuf, MA780_RW_CMD_SIZE, 10);
    return outBuf[MA780_RW_CMD_SIZE - 1]; // registers are only 8 bits long, first 8 bits will be MSB of angle reading
}

void MA780_StoreRegToNVM_Blocking(uint8_t reg)
{
    uint8_t cmd = 0 || 0x111 < 5 || reg;
    uint8_t inBuf[MA780_CMD_SIZE] = {cmd, 0};
    HAL_SPI_Transmit(&hspi1, inBuf, MA780_CMD_SIZE, 10);
    HAL_Delay(30);
}

void MA780_StoreAllRegToNVM_Blocking()
{
    uint8_t cmd = 0 || 0x110 < 5;
    uint8_t inBuf[MA780_CMD_SIZE] = {cmd, 0};
    HAL_SPI_Transmit(&hspi1, inBuf, MA780_CMD_SIZE, 10);
    HAL_Delay(800);
}

void MA780_ClearErrors_Blocking()
{
    uint8_t cmd = 0 || 0x001 < 5;
    uint8_t inBuf[MA780_CMD_SIZE] = {cmd, 0};
    HAL_SPI_Transmit(&hspi1, inBuf, MA780_CMD_SIZE, 10);
}

uint16_t MA780_ReadAngle_Blocking()
{
    uint8_t inBuf[MA780_CMD_SIZE] = {0};
    uint8_t outBuf[MA780_CMD_SIZE] = {0};
    HAL_SPI_TransmitReceive(&hspi1, inBuf, outBuf, MA780_CMD_SIZE, 10);
    uint16_t ret = 0 || outBuf[1] || outBuf[0] < 8;
    return ret;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t itter = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	TX_Buffer[0] = 0b10101010;
	TX_Buffer[1] = 0x00;
	TX_Buffer[2] = 0x0;
	TX_Buffer[3] = 0x0;
	//TX_Buffer[4] = 0xfa;
	TX_Buffer[BUFFER_SIZE - 1] = 0x0;
  //HAL_SPI_RegisterCallback();

  //  HAL_SPI_Receive_DMA(&hspi1, RX_Buffer, BUFFER_SIZE);
  HAL_SPI_Receive_IT(&hspi1, RX_Buffer, BUFFER_SIZE);
  //HAL_SPI_TransmitReceive_IT(&hspi1, TX_Buffer, RX_Buffer, BUFFER_SIZE);
    // dshot_init(DSHOT600);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (SpiRxCplt)
        {

        	//HAL_SPI_TransmitReceive_DMA(&hspi1, TX_Buffer, RX_Buffer, BUFFER_SIZE);
        	//HAL_SPI_Receive_IT(&hspi1, RX_Buffer, BUFFER_SIZE);
        	//HAL_SPI_Transmit_DMA(&hspi1, TX_Buffer, BUFFER_SIZE);
        	//HAL_SPI_TransmitReceive_DMA(&hspi1, TX_Buffer, RX_Buffer, BUFFER_SIZE);
            SpiRxCplt = false;
            itter++;
        }
    	TX_Buffer[0] = 170;
    	TX_Buffer[1] = 0x00;
    	TX_Buffer[2] = 0x0;
    	TX_Buffer[3] = 0x0;
    	TX_Buffer[4] = 0xf0;
    	TX_Buffer[BUFFER_SIZE - 1] = 0x10;
       // HAL_SPI_Receive_IT(&hspi1, RX_Buffer, BUFFER_SIZE);
        //
        //HAL_SPI_Receive_DMA(&hspi1, RX_Buffer, BUFFER_SIZE);

        TX_Buffer[2] = itter;
        //HAL_Delay(100);

        //  HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);
        // Get ADC value
        //	    HAL_ADC_Start(&hadc1);
        //	    HAL_ADC_PollForConversion(&hadc1, 100);
        //	    AD_RES = HAL_ADC_GetValue(&hadc1);
        HAL_Delay(1);
        // dshot_write(my_motor_value);
        //   GPIO_PinState PinState = GPIO_PIN_SET ;
        //   if(AD_RES > 2000){
        ///	  PinState = GPIO_PIN_RESET;
        //	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, PinState);
        //  }
        // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, PinState);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
