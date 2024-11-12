/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <RFDriver_Si4432.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <Timer_milliseconds.h>

#define TRANSMIT
//#define RECEIVE

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LOG UART BUFFER VARIABLE */
char		msg[500]	=	{0};

/* HEART BEAT LED VARIABLES BEGINS */
uint32_t	heartBeatEpoch		=	0,
			heartBeatDuration	=	750;
/* HEART BEAT LED VARIABLES ENDS */

/* --------------------- [RF Variables STARTs] ---------------------------*/

uint32_t	rfModuleBatteryLevelEpoch		=	0,
			rfModuleBatteryLevelDuration	=	10;
/* ---------------------- [RF Variables ENDs] ----------------------------*/



///*---------------------------------- [LOG UART Function STARTs] -----------------------------------*/
///
UART_HandleTypeDef* LOGUart;
void LOGInit(UART_HandleTypeDef* huart)
{
	LOGUart = huart;
}
/// LOG Function.
void LOG(char* buffer)
{
	HAL_UART_Transmit(LOGUart, (const uint8_t*)buffer, strlen(buffer), 1000);
	memset(buffer, 0, strlen(buffer));
}
///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [LOG UART Function ENDs] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LOGInit(&debugUart);

    /** Initialize RF Driver Module **/
    RFDriver_Si4432_begin(	&RF_SPI,
  		  	  	  	  		RF_nSEL_Si4432_GPIO_Port, 	RF_nSEL_Si4432_Pin,
							RF_SDN_Si4432_GPIO_Port,	RF_SDN_Si4432_Pin,
							RF_nIRQ_Si4432_GPIO_Port,	RF_nIRQ_Si4432_Pin
    	  	  	  	  	  );

    #if !defined(noDelay)
    HAL_Delay(100);
    #endif

    /**	Use this function to Set Frequency	**/
    /**	By Default it is Set to 433 MHz		**/
    //RF.setFrequency(433);

    /** Read Frequency Settings **/
    float freq		=	0;
    float freqOffset	=	0;
    readFrequencySettings(&freq, &freqOffset);
    sprintf(	msg,
  			"Frequency is Set: %ld.%ld \t\t Frequency Offset: %ld\r\n",
  			((uint32_t)(freq)),
  			((uint32_t)(freq*1000)%1000),
  			(int32_t)freqOffset
    );
    LOG(msg);

  ///*------------------------------- [ RF Module Testing One Time Run Code BEGINs ] -----------------------------*/
     #ifdef TRANSMIT
        uint32_t txTime	=	0;
     #endif

     #ifdef RECEIVE
        uint32_t rxTime	=	0;
     #endif

  ///*------------------------------- [ RX Part Testing One Time Run Code BEGINs ] -----------------------------*/
  startListening();
  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RX Part Testing One Time Run Code ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RF Module One Time Run Code Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ///*-------------------------------- [ Heart Beat LED BEGINs ] -------------------------------*/
	  if(timerCompleted(&heartBeatEpoch, &heartBeatDuration))
	  {
		  HAL_GPIO_TogglePin(heartBeatLed_GPIO_Port, heartBeatLed_Pin);
		  HAL_Delay(0);
	  }
	  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [ Heart Beat LED ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
	  ///
	  ///*------------------------------- [ RF Module Testing BEGINs ] -----------------------------*/
	  //		if(timerCompleted(&rfModuleBatteryLevelEpoch, &rfModuleBatteryLevelDuration))
	  //		{
	  //			uint8_t devType = 0, devVersion = 0;
	  //			uint8_t batLevel = 0.0;
	  //			RF.getDeviceType(&devType);
	  //			RF.getDeviceVersion(&devVersion);
	  //			RF.getBatteryLevel(&batLevel);
	  ////			uint16_t battLevel = (uint16_t)(batLevel * 100);
	  //			sprintf(msg,
	  //						"Battery Level: %d %% \t Device Type ID: 0x%x \t Device Version: 0x%x\r\n",
	  //						(uint8_t)(batLevel) ,
	  //						devType, devVersion
	  //					);
	  //			LOG(msg);
	  //		}
	  ///*------------------------------- [ TX Part Testing BEGINs ] -----------------------------*/
	  #ifdef TRANSMIT
	  if(timerCompleted(&rfModuleBatteryLevelEpoch, &rfModuleBatteryLevelDuration))
	  {
		  uint8_t txFifoData[]	=	{ 	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
				  0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
				  0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
				  0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
		  };
		  uint8_t rxFifoData[64]	=	{ 0 };
		  uint8_t rxFifoSize	=	0;
		  _Bool txResponse = sendDataPacket_getResponse(txFifoData, sizeof(txFifoData), SET, 500, rxFifoData, &rxFifoSize);
		  if(txResponse)
		  {
			  uint8_t txRSSI	=	0;
			  getRSSI(&txRSSI);
			  sprintf(	msg,
					  "\n-----Data Received-----\n RSSI: %d dB\n Time Duration: %lu\nData: \r\n ",
					  txRSSI,
					  (HAL_GetTick()	-	txTime)
			  ); LOG(msg);

			  uint8_t size = 0;

			  for(uint8_t i = 0; i < rxFifoSize; i++)
			  {
				  char str[20];
				  size_t len = sprintf(str, "%02X, ", rxFifoData[i]);
				  for(uint8_t j = 0; j < len; j++)
					  msg[(uint8_t)size+j]	=	str[j];
				  size+=(uint8_t)len;
			  }
			  LOG(msg);
			  sprintf(	msg,
					  "\n\r\n"
			  );LOG(msg);

			  txTime	=	HAL_GetTick();

		  }
		  else
		  {
			  sprintf(	msg,
					  "Not Acknowledged! \r\n "
			  );
			  LOG(msg);
		  }
	  }
	  #endif /* TRANSMIT */
	  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [TX Part Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


	  ///*------------------------------- [ RX Part Testing BEGINs ] -----------------------------*/
	  #ifdef RECEIVE

	  //			HAL_Delay(0);
	  _Bool rxResponse = isDataPacketReceived();
	  if(rxResponse)
	  {
		  uint8_t rxData[64]	=	{ 0 };
		  uint8_t rxDataSize	=	0;
		  getReceivedDataPacket(rxData, &rxDataSize);
		  uint8_t rxRSSI	=	0;
		  getRSSI(&rxRSSI);
		  sprintf(	msg,
				  "\n-----Data Received-----\n RSSI: %d dB\n Time Duration: %lu\nData: \r\n ",
				  rxRSSI,
				  (HAL_GetTick()	-	rxTime)
		  ); LOG(msg);

		  uint8_t size = 0;

		  for(uint8_t i = 0; i < rxDataSize; i++)
		  {
			  char str[20];
			  size_t len = sprintf(str, "%02X, ", rxData[i]);
			  for(uint8_t j = 0; j < len; j++)
				  msg[(uint8_t)size+j]	=	str[j];
			  size+=(uint8_t)len;
		  }	LOG(msg);


		  rxTime	=	HAL_GetTick();

		  uint8_t txResponseData[]	=	{ 0xAE, 0x26, 0xD8, 0x93 };
		  //							{ 	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
		  //								0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
		  //								0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
		  //								0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f
		  //							};

		  //HAL_Delay(200);
		  while(!sendDataPacket(txResponseData, sizeof(txResponseData)));// HAL_Delay(1);

		  sprintf(
				  msg,
				  "\n^^^^^ Response Sent ^^^^^\r\n "
		  ); LOG(msg);

		  startListening();
	  }
	  #endif /* RECEIVE */
	  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RX Part Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



	  ///*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ [RF Module Testing ENDs ] ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_SDN_Si4432_Pin|RF_nSEL_Si4432_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(heartBeatLed_GPIO_Port, heartBeatLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RF_SDN_Si4432_Pin */
  GPIO_InitStruct.Pin = RF_SDN_Si4432_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SDN_Si4432_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_nIRQ_Si4432_Pin */
  GPIO_InitStruct.Pin = RF_nIRQ_Si4432_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_nIRQ_Si4432_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_nSEL_Si4432_Pin */
  GPIO_InitStruct.Pin = RF_nSEL_Si4432_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_nSEL_Si4432_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : heartBeatLed_Pin */
  GPIO_InitStruct.Pin = heartBeatLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(heartBeatLed_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
