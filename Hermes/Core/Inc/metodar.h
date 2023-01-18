/*
 * metodar.h
 *
 *  Created on: 14. feb. 2022
 *      Author: tagem
 */
// ******************************
// Define
// ******************************
#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS                        4.3f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25                                1430.0f
#define ADC_REFERENCE_VOLTAGE_MV                                    3000.0f
#define ADC_MAX_OUTPUT_VALUE                                        4095.0f
#define TEMP110_CAL_VALUE                                           ( (uint16_t*)( (uint32_t) 0x1FFFF7C2 ) )
#define TEMP30_CAL_VALUE                                            ( (uint16_t*)( (uint32_t) 0x1FFFF7B8 ) )
#define TEMP110                                                     110.0f
#define TEMP30                                                      30.0f

#ifndef INC_METODAR_H_
#define INC_METODAR_H_

void setPWM(TIM_TypeDef* timer, uint8_t kanal, uint32_t frekvens, uint8_t dutyCycle);
int32_t skalerVerdi(int32_t inn, int32_t innMax, int32_t innMin, int32_t utMax, int32_t utMin);
float skalerVerdif(float inn, float innMax, float innMin, float utMax, float utMin);

/*
 * Flash
 */

#define dataIFlash 0xEE
#define FLASH_START 0x80E0000

struct regulatorType {
	union {
		float avstand;
		uint32_t avstand_u;
	};
	// p og p_u har samme minne adresse, brukes for å hente ut float som uint.
	union {
		float p;
		uint32_t p_u;
	};
	union {
		float i;
		uint32_t i_u;
	};
	union {
		float d;
		uint32_t d_u;
	};
	union {
		float respons;
		uint32_t respons_u;
	};
	union {
		float tau;
		uint32_t tau_u;
	};
	uint32_t type;
};
struct filterType {
	uint32_t type;
	uint32_t iir_aks_a;
	uint32_t iir_a;
	uint32_t fir;
};
typedef struct {
	struct regulatorType reg;
	struct filterType filter;
} variabelStructType;

/*
 * Flash slutt
 */

#endif /* INC_METODAR_H_ */
