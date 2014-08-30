/*
 * Displays.h
 *
 *  Created on: Aug 25, 2014
 *      Author: mikrysoft
 */

#ifndef DISPLAYS_H_
#define DISPLAYS_H_

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

//Active high
#define PCF_DIGIT_A 0x4Eu
#define PCF_DIGIT_B 0x4Cu

//Active low
#define PCF_SEGMENT 0x4Au

#define PCF_LED1_A 0x20u
#define PCF_LED1_B 0x00u

#define PCF_LED2_A 0x00u
#define PCF_LED2_B 0x40u

//TOP:		T5 T4 T3 T2 T1 T0
//BOTTOM:	B5 B4 B3 B2 B1 B0
//PCF_A:	T4 T3 L1 B5 T0 T1 T2 T5
//PCF_B:	B1 L2 B2 B0 xx xx B3 B4

#define PCF_T5_A 0x01u
#define PCF_T5_B 0x00u
#define PCF_T4_A 0x80u
#define PCF_T4_B 0x00u
#define PCF_T3_A 0x40u
#define PCF_T3_B 0x00u
#define PCF_T2_A 0x02u
#define PCF_T2_B 0x00u

#define PCF_T1_A 0x04u
#define PCF_T1_B 0x00u
#define PCF_T0_A 0x08u
#define PCF_T0_B 0x00u

#define PCF_B5_A 0x10u
#define PCF_B5_B 0x00u
#define PCF_B4_A 0x00u
#define PCF_B4_B 0x01u
#define PCF_B3_A 0x00u
#define PCF_B3_B 0x02u
#define PCF_B2_A 0x00u
#define PCF_B2_B 0x20u
#define PCF_B1_A 0x00u
#define PCF_B1_B 0x80u
#define PCF_B0_A 0x00u
#define PCF_B0_B 0x10u

/*
 * DISPLAYS: EFKCDBGA
 *   -A-
 * F|   |B
 *   -G-
 * E|   |C
 *   -D-  .K
#define SEG_A 0xFE
 */

#define SEG_A 0x01u
#define SEG_B 0x04u
#define SEG_C 0x10u
#define SEG_D 0x08u
#define SEG_E 0x80u
#define SEG_F 0x40u
#define SEG_G 0x02u
#define SEG_K 0x20u

#define SEG_DIGIT_0		0x22u
#define SEG_DIGIT_1		0xEBu
#define SEG_DIGIT_2		0x70u
#define SEG_DIGIT_3		0xE0u
#define SEG_DIGIT_4		0xA9u
#define SEG_DIGIT_5		0xA4u
#define SEG_DIGIT_6		0x24u
#define SEG_DIGIT_7		0xEAu
#define SEG_DIGIT_8		0x20u
#define SEG_DIGIT_9		0xA0u
#define SEG_DIGIT_A		0x28u
#define SEG_DIGIT_B		0x25u
#define SEG_DIGIT_C		0x75u
#define SEG_DIGIT_CC	0x36u
#define SEG_DIGIT_D		0x61u
#define SEG_DIGIT_E		0x34u
#define SEG_DIGIT_F		0x3Cu
#define SEG_DIGIT_G		0x26u
#define SEG_DIGIT_H		0x2Du
#define SEG_DIGIT_CH	0x29u
#define SEG_DIGIT_I		0x7Fu
#define SEG_DIGIT_CI	0x3Fu
#define SEG_DIGIT_J		0x63u
#define SEG_DIGIT_L		0x37u
#define SEG_DIGIT_N		0x6Du
#define SEG_DIGIT_CN	0x2Au
#define SEG_DIGIT_M		0x6Eu
#define SEG_DIGIT_O		0x65u
#define SEG_DIGIT_CO	0x22u
#define SEG_DIGIT_P		0x38u
#define SEG_DIGIT_Q		0xA8u
#define SEG_DIGIT_R		0x7Du
#define SEG_DIGIT_S		0xA4u
#define SEG_DIGIT_T		0x35u
#define SEG_DIGIT_CT	0x3Eu
#define SEG_DIGIT_U		0x67u
#define SEG_DIGIT_CU	0x23u
#define SEG_DIGIT_V
#define SEG_DIGIT_W		0xB3u
#define SEG_DIGIT_Y		0x39u
#define SEG_DIGIT_Z		0x70u
#define SEG_DIGIT_DOT	0xDFu
#define SEG_DIGIT_TOP	0xFEu
#define SEG_DIGIT_BOT	0xF7u
#define SEG_DIGIT_MID	0xFDu
#define SEG_BLANK		0xFFu

const uint8_t LED_LETTER[42]= {	SEG_DIGIT_A,		SEG_DIGIT_B,	SEG_DIGIT_C,		SEG_DIGIT_D,	SEG_DIGIT_E,	SEG_DIGIT_F,	SEG_DIGIT_G,
								SEG_DIGIT_H,		SEG_DIGIT_I,	SEG_DIGIT_J,		SEG_BLANK/*k*/,	SEG_DIGIT_L,	SEG_DIGIT_M,	SEG_DIGIT_N,
								SEG_DIGIT_O,		SEG_DIGIT_P,	SEG_DIGIT_O/*q*/,	SEG_DIGIT_R,	SEG_DIGIT_S,	SEG_DIGIT_T,	SEG_DIGIT_CU /*U*/,
								SEG_DIGIT_U /*V*/,	SEG_DIGIT_W,	SEG_BLANK/*x*/,		SEG_DIGIT_Y,	SEG_DIGIT_Z,	SEG_DIGIT_MID,	SEG_DIGIT_BOT,
								SEG_DIGIT_DOT,	SEG_BLANK};

const uint8_t LED_DIGIT[16] = {SEG_DIGIT_0, SEG_DIGIT_1, SEG_DIGIT_2, SEG_DIGIT_3, SEG_DIGIT_4, SEG_DIGIT_5, SEG_DIGIT_6, SEG_DIGIT_7, SEG_DIGIT_8, SEG_DIGIT_9, SEG_DIGIT_A, SEG_DIGIT_B, SEG_DIGIT_CC, SEG_DIGIT_D, SEG_DIGIT_E, SEG_DIGIT_F };

typedef struct {
	unsigned int active :1;
	unsigned int dot	:1;
	unsigned int		:6;
	unsigned int value	:8;
} DisplayState;

typedef struct {
	uint8_t addr_a;
	uint8_t addr_b;
} LedAddr;

#define LED_TOP 0
#define LED_BOTTOM 1

#define LED_ROWS 2
#define LED_COLS 6

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

DisplayState display[2][6] =	{
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

void UpdateDisplay(I2C_TypeDef* I2Cx);
void SetDigit(uint8_t row, uint8_t column, uint8_t value, uint8_t active = 1, uint8_t dot = 0);

void SetDot(uint8_t row, uint8_t column);
void ResetDot(uint8_t row, uint8_t column);

rt
#endif /* DISPLAYS_H_ */