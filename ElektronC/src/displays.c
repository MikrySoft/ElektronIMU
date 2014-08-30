/*
 * displays.c
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */


#include "displays.h"
#include "tm_stm32f4_i2c.h"

void UpdateDisplay(I2C_TypeDef* I2Cx)
{
	uint8_t r,c,a,b,p;
	LedAddr displays[8] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
	for (r=0;r<2;r++)
	{
		for (c=0;c<6;c++)
		{
			a = ~LED_DIGIT[display[r][c].value];
			a |= display[r][c].dot ? SEG_K : 0;
			a = display[r][c].active ? a : 0;
			for (b=0;b<8;b++)
			{
				displays[b].addr_a |= (a & (1<<b))? LED_DISP[r][c].addr_a:0;
				displays[b].addr_b |= (a & (1<<b))? LED_DISP[r][c].addr_b:0;

			}
		}
	}
	for (b=0;b<8;b++)
	{
		TM_I2C_Write(I2Cx,PCF_SEGMENT,SEG_BLANK);
		TM_I2C_Write(I2Cx,PCF_DIGIT_A,displays[b].addr_a);
		TM_I2C_Write(I2Cx,PCF_DIGIT_B,displays[b].addr_b);
		TM_I2C_Write(I2Cx,PCF_SEGMENT,~(1<<b));
		msleep(1);
	}
}
