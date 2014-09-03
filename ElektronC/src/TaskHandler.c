/*
 * TaskHandler.c
 *
 *  Created on: Aug 31, 2014
 *      Author: mikrysoft
 */

#include "TaskHandler.h"

volatile STM_Task tasks[MAX_TASK];
uint8_t currentTasks;

int STM_AddTask(uint32_t delay, void (*func)(void))
{
	uint8_t i;
	if (currentTasks == MAX_TASK) return -1;
	for (i=0;i<MAX_TASK;i++)
	{
		if (tasks[i].exists==0)
		{
			tasks[i].timer 		= delay;
			tasks[i].preload 	= delay;
			tasks[i].exists 	= 1;
			tasks[i].func 		= func;
			tasks[i].trigger 	= 0;
			tasks[i].active 	= 1;
			currentTasks ++;
			return i;
		}
	}
	return -2;
}

void STM_DelTask(uint8_t index)
{
	if (index >= MAX_TASK) return;
	tasks[index].active = 0;
	tasks[index].exists = 0;
	tasks[index].trigger = 0;
	tasks[index].preload = 0;
	tasks[index].timer = 0;
	tasks[index].func = (void *) 0;
}

int STM_EnableTask(uint8_t index)
{
	if (index >= MAX_TASK) return -1;
	if (tasks[index].exists)
	{
		if (!tasks[index].active)
		{
			tasks[index].active = 1;
			return 0;
		}	else
		{
			return -1;
		}
	}
	else return -2;
}

int STM_DisableTask(uint8_t index)
{
	if (tasks[index].exists)
	{
		if (tasks[index].active)
		{
			tasks[index].active = 0;
			return 0;
		}	else
		{
			return -1;
		}
	}
	else return -2;
}

void STM_InitTasks(void)
{
	uint8_t task;
	for (task = 0;task<MAX_TASK;task++)
	{
		tasks[task].active = 0;
		tasks[task].exists = 0;
		tasks[task].trigger = 0;
		tasks[task].preload = 0;
		tasks[task].timer = 0;
		tasks[task].func = (void *) 0;
	}
}

void CustomSysTickHandler(void)
{
	uint8_t task;
	for (task = 0; task < MAX_TASK;task++)
	{
		if (tasks[task].active && (!(tasks[task].trigger)))
		{
			if (tasks[task].timer)
			{
				tasks[task].timer-=(1000000/SYSTICK_FREQUENCY_HZ);
			}
			else
			{
				tasks[task].timer = tasks[task].preload;
				tasks[task].trigger = 1;
			}
		}
	}
}

void STM_ExecuteTasks(void)
{
	uint8_t task;
	for (task = 0; task<MAX_TASK;task++)
	{
		if (tasks[task].trigger)
		{
			tasks[task].trigger = 0;
			(tasks[task].func)();
		}
	}
}
