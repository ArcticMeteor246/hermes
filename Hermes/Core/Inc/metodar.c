/*
 * metodar.c
 *
 *  Created on: 14. feb. 2022
 *      Author: tagem
 */

// ******************************
// Inkluderinger
// ******************************
#include "main.h"
#include <math.h>
#include "metodar.h"
#include <string.h>


// ******************************
// Metoder
// ******************************
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
	utVerdi = lroundf( (float) ( (utMax - utMin) * (inn - innMin) ) / (float) (innMax - innMin) ) + utMin;

	// Sjekke øvre og nedre grense på ut verdi
	if (utVerdi > utMax) {
		utVerdi = utMax;
	} else if (utVerdi < utMin) {
		utVerdi = utMin;
	}

	return utVerdi;
}

float skalerVerdif(float inn, float innMax, float innMin, float utMax, float utMin) {
	float utVerdi = 0;
	// Sjekke øvre og nedre gense på inn verdi
	if (inn > innMax) {
		inn = innMax;
	} else if (inn < innMin) {
		inn = innMin;
	}

	// Skaleringsformel
	utVerdi = ( ( (utMax - utMin) * (inn - innMin) ) / (innMax - innMin) ) + utMin;

	// Sjekke øvre og nedre grense på ut verdi
	if (utVerdi > utMax) {
		utVerdi = utMax;
	} else if (utVerdi < utMin) {
		utVerdi = utMin;
	}

	return utVerdi;
}

/*
 * Flash
 */

uint32_t userConfig[32] __attribute__ ((section(".user_data")));
variabelStructType varStruct = { {
		.p_u = 123,
		.i_u = 250,
		.d_u = 560}
};
FLASH_EraseInitTypeDef bank0 = {.TypeErase = FLASH_TYPEERASE_SECTORS, .Sector = FLASH_SECTOR_11, .NbSectors = 1, .VoltageRange = FLASH_VOLTAGE_RANGE_3};

void skrivFlash(variabelStructType* varS) {
	// Bruker flash starter på FLASH_START (0x80E 0000) og er 128KB lang.
	// Ved skriving må flash først nullstilles, Flash sector 11 er område frå 0x80E 0000 til 80F FFFF
	uint32_t error = 0;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&bank0, &error);

	// Er det data i flash som skal hentes inn ved neste boot?
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_START, dataIFlash);

	// Loop gjennom struct
	uint32_t* tempStructPointer = (uint32_t*) varS;
	uint32_t* tempAdrPointer = (uint32_t*) FLASH_START ;
	tempAdrPointer++;
	for (uint8_t i = 0; i < sizeof(variabelStructType); i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) tempAdrPointer, (uint32_t) *tempStructPointer);
		tempAdrPointer++;
		tempStructPointer++;
	}

	HAL_FLASH_Lock();
}

void hentFlash(variabelStructType* varS) {
	if (userConfig[0] == dataIFlash) {
		memcpy(varS, &userConfig[1], sizeof(variabelStructType) );
	} else {
		// FEIL!
	}
}

/*
 * Flash ferdig
 */


/*float lesTemperatur(void) {
	// WIP Defekt ???
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		float sensorVerdi = (float) HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		//float sensorVerdimV = (ADC_REFERENCE_VOLTAGE_MV * sensorVerdi) / ADC_MAX_OUTPUT_VALUE;
		//float sensorVerdiC = ( ((float) TEMP_SENSOR_VOLTAGE_MV_AT_25 - sensorVerdimV) / (float) TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS ) + 25.0f;
		return ((TEMP110 - TEMP30) / ((float)(*TEMP110_CAL_VALUE) - (float)(*TEMP30_CAL_VALUE)) * (sensorVerdi - (float)(*TEMP30_CAL_VALUE)) + TEMP30);
	} else {
		return -273.15f; // 0K = -273.15C
	}
}*/
