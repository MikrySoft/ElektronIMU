#include "stdio.h"
#include "stdlib.h"

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_it.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_usart.h"
#include "hardware_init.h"
#include "main.h"
#include "LSM9DS0.h"
#include "displays.h"
#include "TaskHandler.h"


void Task_UpdateDisplay(void)
{
	UpdateDisplay(I2C2);
}

int main(void){
	uint8_t i,j,k;
	char temp[80];

	LED_init();
	TM_I2C_Init(I2C2,TM_I2C_PinsPack_1,TM_I2C_CLOCK_STANDARD);
	TM_I2C_Init(I2C1,TM_I2C_PinsPack_1,TM_I2C_CLOCK_FAST_MODE);
	TM_USART_Init(USART1,TM_USART_PinsPack_1,115200);
	SysTick_Init();
	i=TM_I2C_ReadReg(I2C1,ACCEL_ADDR, 0x0F);
	j=TM_I2C_ReadReg(I2C1,GYRO_ADDR,0x0F);
	sprintf(temp,"ACCEL: %i (%x)\tGYRO: %i (%x)\r\n",i,i,j,j);
	TM_USART_Puts(USART1,temp);
	STM_InitTasks();
	STM_AddTask(1000,&Task_UpdateDisplay);
	for (k=0;k<4;k++)
	{
		SetDigit(0,5-k,GetDigit(i,10,k),1,0);
		SetDigit(1,5-k,GetDigit(j,10,k),1,0);
	}
	for (k=0;k<2;k++)
	{
		SetDigit(0,1-k,GetDigit(i,16,k),1,0);
		SetDigit(1,1-k,GetDigit(j,16,k),1,0);
	}

	while (1)
	{
		STM_ExecuteTasks();
	}
	return 0;
}
