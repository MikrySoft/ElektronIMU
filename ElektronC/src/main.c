#include "stdio.h"
#include "stdlib.h"

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
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

void Task_ReadTemperature(void)
{
	int16_t temp = LSM9DS0_ReadTemp(I2C1);
	char tekst[32];
	ShowNumber(DISP_TOP,temp);
	sprintf(tekst,"Temp: %d\r\n",temp);
	TM_USART_Puts(USART1,tekst);
}


int main(void){
	LED_init();
	TM_I2C_Init(I2C2,TM_I2C_PinsPack_1,TM_I2C_CLOCK_STANDARD);
	TM_I2C_Init(I2C1,TM_I2C_PinsPack_1,TM_I2C_CLOCK_FAST_MODE);
	TM_USART_Init(USART1,TM_USART_PinsPack_1,115200);

	LSM9DS0_AccMInit(I2C1,LSM9DS0_ACC_FULL_SCALE_2G,LSM9DS0_ACC_AODR_1600HZ,LSM9DS0_ACC_ANTI_ALIAS_BW_733HZ,LSM9DS0_MAG_FULL_SCALE_8G,LSM9DS0_MAG_DATA_RATE_100HZ);

	SysTick_Init();
	STM_InitTasks();
	STM_AddTask(1000,&Task_UpdateDisplay);
	STM_AddTask(20000,&Task_ReadTemperature);
	while (1)
	{
		STM_ExecuteTasks();
	}
	return 0;
}
