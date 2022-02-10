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

#include "tm_lib/tm_stm32_onewire.h"
#include "tm_lib/tm_stm32_ds18b20.h"
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setPWM(TIM_TypeDef* timer, uint8_t kanal, uint32_t frekvens, uint8_t dutyCycle) {
	// frekvens i Hz, dutyCycle i prosent.
	if (frekvens > 2000) {
		timer->PSC = 0;
		if (frekvens <=200000) {
			timer->ARR = (84000000 / frekvens) - 1; // ARR + 1 = 84M / (PSC+1 * frekvens)
		} else {
			timer->ARR = 359; // 200 000Hz med 0 i Prescale
		}
	} else {
		timer->PSC = 839;
		if (frekvens > 5) {
			timer->ARR = (100000 / frekvens) - 1; // ARR + 1 = 84M / (PSC+1 * frekvens)
		} else {
			timer->ARR = 19999; // 5Hz med 719 i Prescale
		}
	}

	uint32_t CCR = (uint32_t) (((float) timer->ARR / (float) 100.0) * (float) dutyCycle);
	switch (kanal) {
	case TIM_CHANNEL_1:
		timer->CCR1 = CCR;
		break;
	case TIM_CHANNEL_2:
		timer->CCR2 = CCR;
		break;
	case TIM_CHANNEL_3:
		timer->CCR3 = CCR;
		break;
	case TIM_CHANNEL_4:
		timer->CCR4 = CCR;
		break;
	}
}

int32_t skalerVerdi(int32_t inn, int32_t innMax, int32_t innMin, int32_t utMax, int32_t utMin) {
	int32_t utVerdi = 0;
	// Sjekke øvre og nedre gense på inn verdi
	if (inn > innMax) {
		inn = innMax;
	} else if (inn < innMin) {
		inn = innMin;
	}

	// Skaleringsformel
	utVerdi = ( ( (utMax - utMin) / (innMax - innMin) ) * (inn - innMin) ) + utMin;

	// Sjekke øvre og nedre grense på ut verdi
	if (utVerdi > utMax) {
		utVerdi = utMax;
	} else if (utVerdi < utMin) {
		utVerdi = utMin;
	}

	return utVerdi;
}

/*
 * CAN-Buss oppsett Start
 */
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer
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
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;

	txHeader.DLC = 8; // Number of bites to be transmitted max- 8
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

TM_OneWire_t OW;

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
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  TIM2->ARR = 1679999; // 50Hz
  TIM2->CCR2 = 84000; // 5% Duty cycle
  //uint8_t Text[] = "Hello\r\n";

  // Read buffer
  uint8_t rxData[16];
  memset(rxData, 0, 16);

  // Read buffer
  //memset(ttData, 97, 13);
  //uint8_t duty = 0;

  oppstartCAN(5, &hcan1);
  TM_OneWire_Init(&OW, TEMP01_GPIO_Port, TEMP01_Pin);
  char buf[40];
  uint8_t devices, i, count;
  //uint8_t alarm_count;
  uint8_t device[8][8];
  //uint8_t alarm_device[8];
  float temps[8];

  count = 0;
  devices = TM_OneWire_First(&OW);
  while (devices) {
	  /* Increase counter */
	  count++;

	  /* Get full ROM value, 8 bytes, give location of first byte where to save */
  	  TM_OneWire_GetFullROM(&OW, device[count - 1]);

	  /* Get next device */
	  devices = TM_OneWire_Next(&OW);
  }
/*   If any devices on 1wire
  	if (count > 0) {
  		sprintf(buf, "Devices found on 1-wire: %d\n", count);
  		TM_USART_Puts(USART1, buf);
  		 Display 64bit rom code for each device
  		for (j = 0; j < count; j++) {
  			for (i = 0; i < 8; i++) {
  				sprintf(buf, "0x%02X ", device[j][i]);
  				TM_USART_Puts(USART1, buf);
  			}
  			TM_USART_Puts(USART1, "\n");
  		}
  	} else {
  		TM_USART_Puts(USART1, "No devices on OneWire.\n");
  	}*/
  /* Go through all connected devices and set resolution to 12bits */
  for (i = 0; i < count; i++) {
	  /* Set resolution to 12bits */
	  TM_DS18B20_SetResolution(&OW, device[i], TM_DS18B20_Resolution_12bits);
  }



/*  //HAL_Delay(2000);
  uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
  while (!CDC_GetRxBufferBytesAvailable_FS()) {

      uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
	    if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
	        while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
      }
  }
  test_flag = 1;*/

  //uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //CDC_Transmit_FS((uint8_t*) &"Start...\n", 9);
	  //CDC_Transmit_FS(ttData, 13);
/*	  TM_DS18B20_StartAll(&OW);
	  while (!TM_DS18B20_AllDone(&OW));

	  for (i = 0; i < count; i++) {
		  if (TM_DS18B20_Read(&OW, device[i], &temps[i])) {
			  sprintf(buf, "Temp %d: %3.5f; \n", i, temps[i]);
			  CDC_Transmit_FS((uint8_t*) buf, strlen(buf) );
		  }
	  }*/
	  uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();

	  // Venter på full pakke, sjekk start og stopp byte.
	  if (bytesAvailable >= 12) {
	  if (CDC_ReadRxBuffer_FS(rxData, 1) == USB_CDC_RX_BUFFER_OK) {
	  if (rxData[0] == 0x02) {
	  if (CDC_ReadRxBuffer_FS(rxData, 11) == USB_CDC_RX_BUFFER_OK) {
	  if (rxData[10] == 0x03) {
		  /*
		   * rxData[0] er CAN/Tilt
		   * rxData[1] er CAN/Kamera ID
		   * rxData[2-9] er Datapakke
		   * rxData[10] er sluttbyte
		   */

		  // 0x05 for CAN, 0x06 for Tilt
		  if (rxData[0] == 0x05) {
			  // Kopierer databuffer og sender på CAN
			  memcpy(csend, &rxData[2], 8);
			  //sendDataCAN(rxData[1], &hcan1);

		  } else {
			  // Her kjem tilt ting

		  }

	  }}}}}
	  /*
	   uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
		    if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
		        while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
	      }*/
/*
	    HAL_Delay(100);
    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    	csend[0] = 'p';
	    sendDataCAN(95, &hcan1);*/

	    //if (HAL_CAN_AddTxMessage(&hcan1,&txHeader,csend,&canMailbox) != HAL_OK) {
	    //}
/*	    if (test_flag) {
	    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	    	//uint8_t Text[16] = "";
	    	//sprintf((char *) Text, "aaaaaaaa", canRX[0]);

	    	memset(Text, '\0', sizeof(Text));
	    	sprintf((char *) Text, " %X", canRX[1]);
	    	while (CDC_Transmit_FS(Text, strlen((const char*) Text)) == USBD_BUSY);

	    	memset(Text, '\0', sizeof(Text));
	    	sprintf((char *) Text, " %X", canRX[2]);
	    	while (CDC_Transmit_FS(Text, strlen((const char*) Text)) == USBD_BUSY);

	    	memset(Text, '\0', sizeof(Text));
	    	sprintf((char *) Text, " %X", canRX[3]);
	    	while (CDC_Transmit_FS(Text, strlen((const char*) Text)) == USBD_BUSY);

	    	memset(Text, '\0', sizeof(Text));
	    	sprintf((char *) Text, "\r\n");
	    	while (CDC_Transmit_FS(Text, strlen((const char*) Text)) == USBD_BUSY);
	    	test_flag = 0;
	    }*/
    	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	  //HAL_Delay(1000);
	  //CDC_Transmit_FS(Text, 7);
	  /*
	  TIM2->CCR2 = skalerVerdi(0, 180, 0, 220000, 50000);
	  HAL_Delay(3000);

	  TIM2->CCR2 = skalerVerdi(180, 180, 0, 220000, 50000);
	  HAL_Delay(3000);

	  TIM2->CCR2 = skalerVerdi(120, 180, 0, 168000, 84000);
	  HAL_Delay(1000);

	  TIM2->CCR2 = skalerVerdi(180, 180, 0, 200000, 60000);
	  HAL_Delay(1000);

	  TIM2->CCR2 = skalerVerdi(120, 180, 0, 200000, 60000);
	  HAL_Delay(1000);

	  TIM2->CCR2 = skalerVerdi(60, 180, 0, 200000, 60000);
	  HAL_Delay(1000);*/

/*	  duty++;
	  HAL_Delay(100);
	  if (duty >= 181) {
		  duty = 0;
	  }*/
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
  sConfigOC.Pulse = 0;
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
  HAL_GPIO_WritePin(TEMP01_GPIO_Port, TEMP01_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

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

	CDC_Transmit_FS(ttData, strlen((char*) ttData));

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
