/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <inttypes.h>

#include "metodar.h"
#include "shift.h"
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

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*
 * CAN-Buss oppsett Start
 */
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t csend[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Tx Buffer
CAN_FilterTypeDef canfilter;
uint32_t canMailbox; //CAN Bus Mail box variable

void oppstartCAN(uint8_t filterGruppe, CAN_HandleTypeDef *canPort) {
	canfilter.FilterBank = 0;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilter.FilterIdHigh = 0x00;
	canfilter.FilterIdLow = 0x00;
	canfilter.FilterMaskIdHigh = 0x00;
	canfilter.FilterMaskIdLow = 0x00;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilter.FilterActivation = DISABLE;
	canfilter.SlaveStartFilterBank = 14;

	txHeader.DLC = 8; // Number of bytes to be transmitted, max- 8
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x00;
	txHeader.ExtId = 0x00;
	txHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_ConfigFilter(canPort, &canfilter);
	HAL_CAN_Start(canPort);
	HAL_CAN_ActivateNotification(canPort, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void sendDataCAN(uint16_t id, CAN_HandleTypeDef *canPort) {
	txHeader.StdId = id;
	HAL_CAN_AddTxMessage(canPort, &txHeader, csend, &canMailbox);
}
/*
 * CAN-Buss oppsett Stopp
 */

uint8_t test_flag = 0;
uint8_t ttData[64] = "123\n";
uint16_t teller_CAN = 0;
uint32_t* tmpCANBuf32;
uint8_t* tmpCANBuf8;
uint8_t CANBufLengde = 0;

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  TIM2->ARR = 1679999;
  TIM2->CCR2 = skalerVerdi(98, 180, 0, 220000, 50000);
  TIM2->CCR3 = skalerVerdi(98, 180, 0, 220000, 50000);

  // Read buffer
  uint8_t rxData[16];
  memset(rxData, 0, 16);

  oppstartCAN(0, &hcan1);

  // Temp verdier til test data
	HAL_GPIO_WritePin(SR_SH_GPIO_Port, SR_SH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_CLCK_INH_GPIO_Port, SR_CLCK_INH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);

  uint8_t dipSwitch = 0;
  uint16_t testSensorTall1 = 0;
  uint16_t testSensorTall2 = 0;
  uint16_t testTeller1 = 0;
  uint8_t  testData[64] = "123\n";
  uint8_t* testBuf8;
  uint8_t  testBufLengde = 0;
  uint8_t  testArray[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // Tx Buffer
  uint8_t  testKontSend[8] = {0};
  int8_t   testKontX = -128;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_HS();

	  // Venter på full pakke, sjekk start og stopp byte.
	  if (bytesAvailable >= 12) {
	  if (CDC_ReadRxBuffer_HS(rxData, 1) == USB_CDC_RX_BUFFER_OK) {
	  if (rxData[0] == PAKKE_START) {
	  if (CDC_ReadRxBuffer_HS(rxData, 11) == USB_CDC_RX_BUFFER_OK) {
	  if (rxData[10] == PAKKE_STOPP) {
		  /*
		   * rxData[0] er CAN/Tilt
		   * rxData[1] er CAN/Kamera ID
		   * rxData[2-9] er Datapakke
		   * rxData[10] er sluttbyte
		   */

		  if (rxData[0] == PAKKE_CAN) {
			  // Kopierer databuffer og sender på CAN
			  memcpy(csend, &rxData[2], 8);
			  sendDataCAN(rxData[1], &hcan1);

		  } else if (rxData[0] == PAKKE_TILT) {

			  int8_t vinkel = (int8_t) rxData[2];
			  int16_t vinkel_16 = (int16_t) vinkel;

			  switch (rxData[1]) {
			  // Kamera fram
			  case 200:
				  vinkel_16 += KAMERA_VINKEL_FRAM; // -30 -> +30 offsett med 90

				  if (vinkel_16 >= KAMERA_VINKEL_FRAM_MAKS) {
					  vinkel_16 = KAMERA_VINKEL_FRAM_MAKS;
				  } else if (vinkel_16 <= KAMERA_VINKEL_FRAM_MIN) {
					  vinkel_16 = KAMERA_VINKEL_FRAM_MIN;
				  }

				  TIM2->CCR2 = skalerVerdi(vinkel_16, 180, 0, 220000, 50000);
				  break;

			  // Kamera bak
			  case 201:
				  vinkel_16 += KAMERA_VINKEL_BAK; // -30 -> +30 offsett med 90

				  if (vinkel_16 >= KAMERA_VINKEL_BAK_MAKS) {
					  vinkel_16 = KAMERA_VINKEL_BAK_MAKS;
				  } else if (vinkel_16 <= KAMERA_VINKEL_BAK_MIN) {
					  vinkel_16 = KAMERA_VINKEL_BAK_MIN;
				  }

				  TIM2->CCR3 = skalerVerdi(vinkel_16, 180, 0, 220000, 50000);
				  break;
			  }

		  } else {

		  }

	  }}}
	  // Ikkje Start byte
	  else {
		  // ting her??
	  }}}

	  	if (testTeller1 >= 0xFFFF) {
		  	dipSwitch = readByte();

		  	if (dipSwitch != 0xFF) {
				// Testdata sensor kort til toppside
				if (dipSwitch & 0b1) {
					if (testSensorTall1 >= 700) {
						testSensorTall1 = 0;
					}
					testSensorTall1++;
					testSensorTall2 = testSensorTall1 + 42;

					memcpy(&testArray[1], &testSensorTall1, 2);
					memcpy(&testArray[3], &testSensorTall2, 2);

					testBuf8 = testArray;
					for (int i = 0; i < 8; i++) {
						testBufLengde = testBufLengde + sprintf((char*) &testData[testBufLengde], "\\x%02X", *testBuf8);
						testBuf8++;
					}

					sprintf((char*) &testData[testBufLengde], ";%d\n", (uint8_t) 140U);
					testBufLengde = 0;

					CDC_Transmit_HS(testData, strlen((char*) testData));
				}

				// Testdata kontroll til CAN
				if (dipSwitch & 0b10) {
				  testKontX++;
				  memset(testKontSend, (uint8_t) testKontX, 3);

				  memcpy(csend, testKontSend, 8);
				  sendDataCAN(70, &hcan1);
				}

				// Debugdata serial konsoll på
				if (dipSwitch & 0b10000000) {
					sprintf((char*) testData, "Dipswitch: 0b"BYTE_TO_BINARY_PATTERN"\r\n", BYTE_TO_BINARY(dipSwitch));
					CDC_Transmit_HS(testData, strlen((char*) testData));
				}
		  	}
		  	// Reset teller
		  	testTeller1 = 0;
	  	}
	  	//testTeller1++;
	  	//sendDataCAN(90, &hcan1);
	  	//HAL_Delay(1000);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1679999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 9999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SR_SH_Pin|SR_CLK_Pin|SR_CLCK_INH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEMP01_GPIO_Port, TEMP01_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SR_OUT_Pin */
  GPIO_InitStruct.Pin = SR_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SR_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SR_SH_Pin SR_CLK_Pin SR_CLCK_INH_Pin */
  GPIO_InitStruct.Pin = SR_SH_Pin|SR_CLK_Pin|SR_CLCK_INH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TEMP01_Pin */
  GPIO_InitStruct.Pin = TEMP01_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEMP01_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Interupt for CAN-Buss mottak av data
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan11)
{
	HAL_CAN_GetRxMessage(hcan11, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	tmpCANBuf8 = canRX;
	for (int i = 0; i < 8; i++) {
		CANBufLengde = CANBufLengde + sprintf((char*) &ttData[CANBufLengde], "\\x%02X", *tmpCANBuf8);
		tmpCANBuf8++;
	}

	sprintf((char*) &ttData[CANBufLengde], ";%d\n", (uint8_t) rxHeader.StdId);
	CANBufLengde = 0;

	CDC_Transmit_HS(ttData, strlen((char*) ttData));
	//CDC_Transmit_FS(ttData, strlen((char*) ttData));

}
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
