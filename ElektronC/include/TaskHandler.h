/*
 * TaskHandler.h
 *
 *  Created on: Aug 31, 2014
 *      Author: mikrysoft
 */

#ifndef TASKHANDLER_H_
#define TASKHANDLER_H_

#include "stm32f4xx.h"

#define MAX_TASK 20
#define SYSTICK_FREQUENCY_HZ	(10000u)

typedef struct {
	uint32_t timer;
	uint32_t preload;
	uint8_t	 trigger	:1;
	uint8_t	 active		:1;
	uint8_t	 exists		:1;
	uint8_t				:5;
	void (* func)(void);
} STM_Task;

int STM_AddTask(uint32_t delay, void (*func)(void));
void STM_DelTask(uint8_t index);
int STM_EnableTask(uint8_t index);
int STM_DisableTask(uint8_t index);
void STM_InitTasks(void);
void CustomSysTickHandler(void);
void STM_ExecuteTasks(void);


#endif /* TASKHANDLER_H_ */
