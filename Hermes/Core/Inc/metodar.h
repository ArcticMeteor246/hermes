/*
 * metodar.h
 *
 *  Created on: 14. feb. 2022
 *      Author: tagem
 */

#ifndef INC_METODAR_H_
#define INC_METODAR_H_

void setPWM(TIM_TypeDef* timer, uint8_t kanal, uint32_t frekvens, uint8_t dutyCycle);
int32_t skalerVerdiI(int32_t inn, int32_t innMax, int32_t innMin, int32_t utMax, int32_t utMin);
uint32_t skalerVerdiU(uint32_t inn, uint32_t innMax, uint32_t innMin, uint32_t utMax, uint32_t utMin);
float skalerVerdiF(float inn, float innMax, float innMin, float utMax, float utMin);

#endif /* INC_METODAR_H_ */
