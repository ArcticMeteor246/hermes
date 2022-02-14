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

int32_t skalerVerdiI(int32_t inn, int32_t innMax, int32_t innMin, int32_t utMax, int32_t utMin) {
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

uint32_t skalerVerdiU(uint32_t inn, uint32_t innMax, uint32_t innMin, uint32_t utMax, uint32_t utMin) {
	uint32_t utVerdi = 0;
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

float skalerVerdiF(float inn, float innMax, float innMin, float utMax, float utMin) {
	float utVerdi = 0;
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
