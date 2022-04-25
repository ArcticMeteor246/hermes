/*
 * shift.c
 *
 *  Created on: Mar 17, 2022
 *      Author: tagem
 */

// ******************************
// Inkluderinger
// ******************************
#include "main.h"


// ******************************
// Metoder
// ******************************

uint8_t readByte() {
	uint8_t out = 0x00;

	HAL_GPIO_WritePin(SR_SH_GPIO_Port, SR_SH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SR_SH_GPIO_Port, SR_SH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_CLCK_INH_GPIO_Port, SR_CLCK_INH_Pin, GPIO_PIN_RESET);

	// Begynn å lese data
	HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
	for (uint8_t i = 0; i < 8; i++) {
		if (HAL_GPIO_ReadPin(SR_OUT_GPIO_Port, SR_OUT_Pin)) {
			out |= (1 << i);
		} else {
			out &= ~(1 << i);
		}
		HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(SR_CLK_GPIO_Port, SR_CLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SR_CLCK_INH_GPIO_Port, SR_CLCK_INH_Pin, GPIO_PIN_SET);

	return out;
}
