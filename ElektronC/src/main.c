#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "stm32f4xx_it.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_usart.h"
#include "displays.h"
#include "hardware_init.h"
#include "main.h"




uint8_t GetDigit(uint32_t number, uint8_t base, uint8_t digit)
{
	while (digit--)
	{
		number /= base;
	}
	return number % base;
}



int main(void){
	uint8_t i,j;
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
	display[0][5].value=GetDigit(i,10,0); 	display[0][5].active=1;
	display[0][4].value=GetDigit(i,10,1); 	display[0][4].active=1;
	display[0][3].value=GetDigit(i,10,2); 	display[0][3].active=1;
	display[0][1].value=GetDigit(i,16,0); 	display[0][1].active=1;
	display[0][0].value=GetDigit(i,16,1); 	display[0][0].active=1;

	display[1][5].value=GetDigit(j,10,0); 	display[1][5].active=1;
	display[1][4].value=GetDigit(j,10,1); 	display[1][4].active=1;
	display[1][3].value=GetDigit(j,10,2); 	display[1][3].active=1;
	display[1][1].value=GetDigit(j,16,0); 	display[1][1].active=1;
	display[1][0].value=GetDigit(j,16,1); 	display[1][0].active=1;

	while (1)
	{
		UpdateDisplay(I2C2);
	}
	return 0;
}
