//minor change to test github link
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
//#include "stdlib.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TextOutBuf[1024];
uint8_t Bbuffer[1024];
static uint8_t tx_buffer[1000];
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured

uint32_t Evt_stack =0; //number of cosmic ray events this second
uint32_t Evt_timestamps[30]; //space for 30 events per second, no overflow as yet!
uint32_t Evt_total = 0; //total events since start of operation
uint32_t gps_timestamp =0; //value of TIM2 when GPS PPS arrives.

uint8_t data_ready=0; //flag for data ready to send to UART1 via DMA.
uint8_t pps_started=0;


void WatchdogRefresh(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

void debugPrint(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}

void debugPrintln(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

//struct __FILE {
//	int dummy;
//};
//declare a file
//FILE __stdout;

//int fputc(int ch, FILE *f){
//	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//	return ch;
//}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

void print_buffer(void)
{
	//TextOutBufSize = strlen(TextOutBuf); //check characters in buffer
	if (strlen(TextOutBuf) >0 )
	{
		//print one if there is a character to print
		//problem here, it only transmits the first letter, over and over
		//see if there's a DMA route?
		//DMA instruction to buffer -> UART would be ideal.
		HAL_UART_Transmit(&huart1, TextOutBuf, 1, 1000);
		//HAL_UART_Transmit(&huart1, TextOutBuf, strlen(TextOutBuf), 1000);
	}
	//if (strlen(TextOutBuf)==0 )
	//{
	//memset(&TextOutBuf[0], 0, sizeof(TextOutBuf));
	//}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HAL_NVIC_DisableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
	//memset(TextOutBuf,0,strlen(TextOutBuf));

	debugPrint(&huart1, "TIM2\r\n");

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		pps_started = 1;
		//here we are in the PPS case. Clear the buffer
		memset(TextOutBuf,0,strlen(TextOutBuf));


		debugPrint(&huart1, "GPSPPS\r\n");
		HAL_GPIO_TogglePin(pwr_led_GPIO_Port,pwr_led_Pin);
		sprintf((char*)TextOutBuf+strlen(TextOutBuf), "PPS: GPS lock:1;\r\n");


		{
			//normal operation mode
			debugPrint(&huart1, "PPS\r\n");
			//oldtimestamp = gps_timestamp; //backup the old value
			gps_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);  // capture the first value

			//HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_SET);

			//here goes the code to do the readouts; write second
			//readout events
			//readout secondary data
			//now reset the counter to 0;
			//print sensors was here, moving to main loop
			//bme_readout();
			//read_imu();
			//avg_temp_print();

			if (Evt_stack > 0) {
				uint16_t evt_ct = Evt_stack;
				while (evt_ct > 0) {
					//for (uint8_t prt_ctr=0; prt_ctr<=(Evt_stack); prt_ctr++)
					//{
					sprintf((char*)TextOutBuf+strlen(TextOutBuf), "Event: sub second micros:%d/%d; Event Count:%d\r\n", Evt_timestamps[evt_ct], gps_timestamp, (Evt_total+evt_ct));
					evt_ct--;
					//sprintf((char*)TextOutBuf, "GPS_PPS\r\n");

				}
			}
			Evt_total = Evt_total+Evt_stack; //increment total events

			//HAL_GPIO_TogglePin(pwr_led_GPIO_Port,pwr_led_Pin);
			//sprintf((char*)TextOutBuf, "GPS_PPS\r\n");
			//HAL_UART_Transmit(&huart1, TextOutBuf, sizeof(TextOutBuf), 1000);
			data_ready=1;
			//reset ctr
			TIM2->CNT = 0; //reset the ctr
			TIM2->CR1 |= 0x01;

			//after we print, set the event stack back to 0;
			Evt_stack=0;

			//TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
			//TIM2->CCER |= TIM_CCER_CC1E;
			//TIM2->CR1 |= TIM_CR1_CEN;
			//TIM2->SR = ~TIM_SR_CC1IF;
		}

	}
	else
		//here we are in the event case. Which is all other times we execute this routine if channel 1 wasn't used.

		//if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if interrput source is channel 2, cosmic event
	{
		HAL_GPIO_WritePin(evt_led_GPIO_Port,evt_led_Pin, GPIO_PIN_SET);

		//debugPrint(&huart1, "evt\r\n");


		//when we have an event, we read the timer into the nth slot of the stack.
		if (pps_started)
		{
			Evt_stack++; // we put event 1 in bin 1, the 0th value of the array is not used.
			Evt_timestamps[Evt_stack] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);  // capture the first value

		}
		HAL_GPIO_WritePin(evt_led_GPIO_Port,evt_led_Pin, GPIO_PIN_SET);
		//if (Evt_stack>30) sprintf((char*)TextOutBuf, "Event overflow");

		//HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_SET); //set event pin, we'll reset it in the main loop after a v. short delay.

	}
	//data_ready=1;
	HAL_NVIC_EnableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //HAL_Delay(1500);
  HAL_GPIO_WritePin(pwr_led_GPIO_Port, pwr_led_Pin, GPIO_PIN_SET);
  debugPrint(&huart1, "Cosmic Pi Version 2 startup \r\n"); // print
  debugPrint(&huart1, "Firmware Version 27/10/24 \r\n");

  debugPrint(&huart1, "Timer init - GPS PPS is needed\r\n");
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1); //gps pps timer routine
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //gps pps timer routine
  //TIM_Cmd(TIM2, ENABLE);
  TIM2->CR1 |= 0x01;
  //debugPrint(&huart1, "timer init - EVT\r\n");
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2); //gps pps timer routine
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //event timer routine
  TIM2->CR1 |= 0x01;

  //HAL_TIM_Base_Start();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  debugPrint(&huart1, "Enable interrupts...\r\n");
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  WatchdogRefresh();
	  HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_RESET);
	  //debugPrint(&huart1, "mainloop\r\n");
	  if (pps_started)
	  		{
		  debugPrint(&huart1, "pps+\r\n");
		  }
	  else
	  {
		  debugPrint(&huart1, "pps-\r\n");
	  }
		if (data_ready) {
			HAL_NVIC_DisableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
			HAL_UART_Transmit(&huart1, TextOutBuf, strlen(TextOutBuf), 100); //send one char at a time when idle.
			WatchdogRefresh();
			HAL_NVIC_EnableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.

			data_ready = 0;
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 42500000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
	//HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RPI_extra_Pin|inj_led_Pin|pwr_led_Pin|evt_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ch_b_or_Pin|ch_a_or_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RPI_extra_Pin inj_led_Pin pwr_led_Pin evt_led_Pin */
  GPIO_InitStruct.Pin = RPI_extra_Pin|inj_led_Pin|pwr_led_Pin|evt_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : flag_Pin */
  GPIO_InitStruct.Pin = flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(flag_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ch_b_or_Pin ch_a_or_Pin */
  GPIO_InitStruct.Pin = ch_b_or_Pin|ch_a_or_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
