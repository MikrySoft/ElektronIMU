/*
 * main.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stm32f4xx.h>

void Task_UpdateDisplay(void);
void Task_ReadTemperature(void);
void Task_BlinkLED(void);
void Task_GetAcceleration(void);


#endif /* MAIN_H_ */
