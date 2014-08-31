/*
 * displays.c
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */


#include "tm_stm32f4_i2c.h"
#include "displays.h"

const LedAddr LED_DISP[LED_ROWS][LED_COLS] 	= {
								{
										{.addr_a=PCF_T5_A,.addr_b=PCF_T5_B},
										{.addr_a=PCF_T4_A,.addr_b=PCF_T4_B},
										{.addr_a=PCF_T3_A,.addr_b=PCF_T3_B},
										{.addr_a=PCF_T2_A,.addr_b=PCF_T2_B},
										{.addr_a=PCF_T1_A,.addr_b=PCF_T1_B},
										{.addr_a=PCF_T0_A,.addr_b=PCF_T0_B}
								},
								{
										{.addr_a=PCF_B5_A,.addr_b=PCF_B5_B},
										{.addr_a=PCF_B4_A,.addr_b=PCF_B4_B},
										{.addr_a=PCF_B3_A,.addr_b=PCF_B3_B},
										{.addr_a=PCF_B2_A,.addr_b=PCF_B2_B},
										{.addr_a=PCF_B1_A,.addr_b=PCF_B1_B},
										{.addr_a=PCF_B0_A,.addr_b=PCF_B0_B}
								}
							   };

const uint8_t LED_LETTER[32]= {	SEG_DIGIT_A,	SEG_DIGIT_B,	SEG_DIGIT_C,	SEG_DIGIT_D,	SEG_DIGIT_E,	SEG_DIGIT_F,	SEG_DIGIT_G,
								SEG_DIGIT_H,	SEG_DIGIT_I,	SEG_DIGIT_J,	SEG_DIGIT_K,	SEG_DIGIT_L,	SEG_DIGIT_M,	SEG_DIGIT_N,
								SEG_DIGIT_O,	SEG_DIGIT_P,	SEG_DIGIT_Q,	SEG_DIGIT_R,	SEG_DIGIT_S,	SEG_DIGIT_T,	SEG_DIGIT_U,
								SEG_DIGIT_V,	SEG_DIGIT_W,	SEG_DIGIT_X,	SEG_DIGIT_Y,	SEG_DIGIT_Z,	SEG_DIGIT_MID,	SEG_DIGIT_BOT,
								SEG_DIGIT_APO,	SEG_DIGIT_DOT,	SEG_BLANK};

const uint8_t LED_DIGIT[16] = {SEG_DIGIT_0, SEG_DIGIT_1, SEG_DIGIT_2, SEG_DIGIT_3, SEG_DIGIT_4, SEG_DIGIT_5, SEG_DIGIT_6, SEG_DIGIT_7, SEG_DIGIT_8, SEG_DIGIT_9, SEG_DIGIT_A, SEG_DIGIT_B, SEG_DIGIT_CC, SEG_DIGIT_D, SEG_DIGIT_E, SEG_DIGIT_F };


DisplayState C96_DISP[2][6] =	{
									{
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0}
									},
									{
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0},
											{.active=0, .dot=0,.value=0}
									}
								};


uint8_t GetDigit(uint32_t number, uint8_t base, uint8_t digit)
{
	while (digit--)
	{
		number /= base;
	}
	return number % base;
}


void UpdateDisplay(I2C_TypeDef* I2Cx)
{
	uint8_t r,c,a,b;
	LedAddr displays[8] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
	for (r=0;r<2;r++)
	{
		for (c=0;c<6;c++)
		{
			a = ~LED_DIGIT[C96_DISP[r][c].value];
			a |= C96_DISP[r][c].dot ? SEG_K : 0;
			a = C96_DISP[r][c].active ? a : 0;
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
	}
}

void SetDigit(uint8_t row, uint8_t column, uint8_t value, uint8_t active, uint8_t dot)
{
	if ((row > 1)||(column>5)||value>15) return;
	C96_DISP[row][column].active = active?1:0;
	C96_DISP[row][column].dot    = dot?1:0;
	C96_DISP[row][column].value  = value;
}

void ShowNumber(uint8_t settings, int32_t number)
{
	uint8_t row = (settings&DISP_TOP);
	uint8_t base = (settings&DISP_HEX)?16:10;
	uint8_t visible = (settings&DISP_FILL);
	uint8_t i,t;
	if (number < 0) {
		number = - number;
		C96_DISP[row][0].active = 1;
		C96_DISP[row][0].value = SEG_DIGIT_MID;
	}
	for (i = 0;i<5;i++)
	{
		t = GetDigit(number,base,4-i);
		visible = (t==0)?visible:1;
		C96_DISP[row][i+1].active = visible;
		C96_DISP[row][i+1].value = t;
	}

}

