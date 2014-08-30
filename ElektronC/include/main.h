/*
 * main.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stm32f4xx.h>

void sleep(uint32_t wait);
void SendDigit(I2C_TypeDef* I2Cx, uint8_t row, uint8_t digit, uint8_t value);
uint8_t GetDigit(uint32_t number, uint8_t base, uint8_t digit);

#endif /* MAIN_H_ */
