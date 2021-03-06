/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

typedef enum {
	PAKKE_START			= 0x02U,
    PAKKE_STOPP			= 0x03U,
	PAKKE_CAN			= 0x05U,
	PAKKE_TILT			= 0x06U
} PAKKE_OPPSETT;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SR_OUT_Pin GPIO_PIN_4
#define SR_OUT_GPIO_Port GPIOA
#define SR_SH_Pin GPIO_PIN_5
#define SR_SH_GPIO_Port GPIOA
#define SR_CLK_Pin GPIO_PIN_6
#define SR_CLK_GPIO_Port GPIOA
#define SR_CLCK_INH_Pin GPIO_PIN_7
#define SR_CLCK_INH_GPIO_Port GPIOA
#define TEMP01_Pin GPIO_PIN_1
#define TEMP01_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_12
#define LD1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define KAMERA_VINKEL_FRAM			98u
#define KAMERA_VINKEL_FRAM_MAKS		(KAMERA_VINKEL_FRAM + 30u)
#define KAMERA_VINKEL_FRAM_MIN		(KAMERA_VINKEL_FRAM - 30u)

#define KAMERA_VINKEL_BAK			98u
#define KAMERA_VINKEL_BAK_MAKS		(KAMERA_VINKEL_BAK + 30u)
#define KAMERA_VINKEL_BAK_MIN		(KAMERA_VINKEL_BAK - 30u)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
