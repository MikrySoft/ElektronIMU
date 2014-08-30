/**
 *	HD44780 LCD driver library for STM32F4xx
 *	It also support's all HD44780 compatible LCD drivers
 *
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@link		http://stm32f4-discovery.com/2014/06/library-16-interfacing-hd44780-lcd-controller-with-stm32f4/
 *	@version 	v1.0
 *	
 *	
 *	Default pinout
 *	
 *	LCD		STM32F4XX		DESCRIPTION
 *	
 *	GND		GND				Ground
 *	VCC		+5V				Power supply for LCD
 *	V0		Potentiometer	Contrast voltage. Connect to potentiometer
 *	RS		PB2				Register select, can be overwritten in your project�s defines.h file
 *	RW		GND				Read/write
 *	E		PB7				Enable pin, can be overwritten in your project�s defines.h file
 *	D0		-				Data 0 � doesn�t care
 *	D1		-				Data 1 - doesn�t care
 *	D2		-				Data 2 - doesn�t care
 *	D3		-				Data 3 - doesn�t  care
 *	D4		PC12			Data 4, can be overwritten in your project�s defines.h file
 *	D5		PC13			Data 5, can be overwritten in your project�s defines.h file
 *	D6		PC14			Data 6, can be overwritten in your project�s defines.h file
 *	D7		PC15			Data 7, can be overwritten in your project�s defines.h file
 *	A		+3V3			Backlight positive power
 *	K		GND				Ground for backlight
 *	
 *	
 *	If you want to change pinout, do this in your defines.h file with lines below and set your own settings:
 *	
 *	//RS - Register select pin
 *	#define TM_HD44780_RS_RCC		RCC_AHB1Periph_GPIOB
 *	#define TM_HD44780_RS_PORT		GPIOB
 *	#define TM_HD44780_RS_PIN		GPIO_Pin_2
 *	//E - Enable pin
 *	#define TM_HD44780_E_RCC		RCC_AHB1Periph_GPIOB
 *	#define TM_HD44780_E_PORT		GPIOB
 *	#define TM_HD44780_E_PIN		GPIO_Pin_7
 *	//D4 - Data 4 pin
 *	#define TM_HD44780_D4_RCC		RCC_AHB1Periph_GPIOC
 *	#define TM_HD44780_D4_PORT		GPIOC
 *	#define TM_HD44780_D4_PIN		GPIO_Pin_12
 *	//D5 - Data 5 pin
 *	#define TM_HD44780_D5_RCC		RCC_AHB1Periph_GPIOC
 *	#define TM_HD44780_D5_PORT		GPIOC
 *	#define TM_HD44780_D5_PIN		GPIO_Pin_13
 *	//D6 - Data 6 pin
 *	#define TM_HD44780_D6_RCC		RCC_AHB1Periph_GPIOC
 *	#define TM_HD44780_D6_PORT		GPIOC
 *	#define TM_HD44780_D6_PIN		GPIO_Pin_14
 *	//D7 - Data 7 pin
 *	#define TM_HD44780_D7_RCC		RCC_AHB1Periph_GPIOC
 *	#define TM_HD44780_D7_PORT		GPIOC
 *	#define TM_HD44780_D7_PIN		GPIO_Pin_15
 */
#ifndef TM_HD44780_H
#define TM_HD44780_H 100
/**
 * Dependencies
 * 	- STM32F4xx
 * 	- STM32F4xx RCC
 * 	- STM32F4xx GPIO
 * 	- defines.h
 * 	- TM DELAY
 */
/**
 * Includes
 */
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "defines.h"
#include "tm_stm32f4_delay.h"

//4 bit mode
//Control pins, can be overwritten
//RS - Register select pin
#ifndef TM_HD44780_RS_PIN
#define TM_HD44780_RS_RCC				RCC_AHB1Periph_GPIOB
#define TM_HD44780_RS_PORT				GPIOB
#define TM_HD44780_RS_PIN				GPIO_Pin_2
#endif
//E - Enable pin
#ifndef TM_HD44780_E_PIN
#define TM_HD44780_E_RCC				RCC_AHB1Periph_GPIOB
#define TM_HD44780_E_PORT				GPIOB
#define TM_HD44780_E_PIN				GPIO_Pin_7
#endif
//Data pins, can be overwritten
//D4 - Data 4 pin
#ifndef TM_HD44780_D4_PIN
#define TM_HD44780_D4_RCC				RCC_AHB1Periph_GPIOC
#define TM_HD44780_D4_PORT				GPIOC
#define TM_HD44780_D4_PIN				GPIO_Pin_12
#endif
//D5 - Data 5 pin
#ifndef TM_HD44780_D5_PIN
#define TM_HD44780_D5_RCC				RCC_AHB1Periph_GPIOC
#define TM_HD44780_D5_PORT				GPIOC
#define TM_HD44780_D5_PIN				GPIO_Pin_13
#endif
//D6 - Data 6 pin
#ifndef TM_HD44780_D6_PIN
#define TM_HD44780_D6_RCC				RCC_AHB1Periph_GPIOC
#define TM_HD44780_D6_PORT				GPIOC
#define TM_HD44780_D6_PIN				GPIO_Pin_14
#endif
//D7 - Data 7 pin
#ifndef TM_HD44780_D7_PIN
#define TM_HD44780_D7_RCC				RCC_AHB1Periph_GPIOC
#define TM_HD44780_D7_PORT				GPIOC
#define TM_HD44780_D7_PIN				GPIO_Pin_15
#endif

#define TM_HD44780_RS_LOW				GPIO_WriteBit(TM_HD44780_RS_PORT, TM_HD44780_RS_PIN, Bit_RESET)
#define TM_HD44780_RS_HIGH				GPIO_WriteBit(TM_HD44780_RS_PORT, TM_HD44780_RS_PIN, Bit_SET)
#define TM_HD44780_E_LOW				GPIO_WriteBit(TM_HD44780_E_PORT, TM_HD44780_E_PIN, Bit_RESET)
#define TM_HD44780_E_HIGH				GPIO_WriteBit(TM_HD44780_E_PORT, TM_HD44780_E_PIN, Bit_SET)

#define TM_HD44780_E_BLINK				TM_HD44780_E_HIGH; TM_HD44780_Delay(20); TM_HD44780_E_LOW; TM_HD44780_Delay(20)
#define TM_HD44780_Delay(x)				Delay(x)

//Commands
#define TM_HD44780_CLEARDISPLAY			0x01
#define TM_HD44780_RETURNHOME			0x02
#define TM_HD44780_ENTRYMODESET			0x04
#define TM_HD44780_DISPLAYCONTROL		0x08
#define TM_HD44780_CURSORSHIFT			0x10
#define TM_HD44780_FUNCTIONSET			0x20
#define TM_HD44780_SETCGRAMADDR			0x40
#define TM_HD44780_SETDDRAMADDR			0x80

//Flags for display entry mode
#define TM_HD44780_ENTRYRIGHT			0x00
#define TM_HD44780_ENTRYLEFT			0x02
#define TM_HD44780_ENTRYSHIFTINCREMENT 	0x01
#define TM_HD44780_ENTRYSHIFTDECREMENT 	0x00

//Flags for display on/off control
#define TM_HD44780_DISPLAYON			0x04
#define TM_HD44780_CURSORON				0x02
#define TM_HD44780_BLINKON				0x01

//Flags for display/cursor shift
#define TM_HD44780_DISPLAYMOVE			0x08
#define TM_HD44780_CURSORMOVE			0x00
#define TM_HD44780_MOVERIGHT			0x04
#define TM_HD44780_MOVELEFT				0x00

//Flags for function set
#define TM_HD44780_8BITMODE				0x10
#define TM_HD44780_4BITMODE				0x00
#define TM_HD44780_2LINE				0x08
#define TM_HD44780_1LINE				0x00
#define TM_HD44780_5x10DOTS				0x04
#define TM_HD44780_5x8DOTS				0x00

/**
 * Internal struct for LCD
 *
 */
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
} TM_HD44780_Options_t;

/**
 * Initialize LCD
 *
 * Parameters:
 * 	- uint8_t cols: width of lcd
 * 	- uint8_t rows: height of lcd
 *
 * No return
 */
extern void TM_HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * Turn display on
 *
 * No return
 */
extern void TM_HD44780_DisplayOn(void);

/**
 * Turn display off
 *
 * No return
 */
extern void TM_HD44780_DisplayOff(void);

/**
 * Clear entire LCD
 *
 * No return
 */
extern void TM_HD44780_Clear(void);

/**
 * Put string on lcd
 *
 * Parameters:
 * 	- uint8_t x: x location
 * 	- uint8_t y: y location
 * 	- char* str: pointer to string
 *
 * No return
 */
extern void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * Enable cursor blink
 *
 * No return
 */
extern void TM_HD44780_BlinkOn(void);

/**
 * Disable cursor blink
 *
 * No return
 */
extern void TM_HD44780_BlinkOff(void);

/**
 * Show cursor
 *
 * No return
 */
extern void TM_HD44780_CursorOn(void);

/**
 * Hide cursor
 *
 * No return
 */
extern void TM_HD44780_CursorOff(void);

/**
 * Scroll display to the left
 *
 * No return
 */
extern void TM_HD44780_ScrollLeft(void);

/**
 * Scroll display to the right
 *
 * No return
 */
extern void TM_HD44780_ScrollRight(void);

/**
 * Create custom character
 *
 * Parameters:
 *  - uint8_t location:
 *  	Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 *  - uint8_t *data
 *  	Pointer to 8bytes of data for one character
 *
 * No return
 */
extern void TM_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * Put custom created character on LCD
 *
 * Parameters:
 * 	- uint8_t location
 * 		Location on LCD where character is stored, 0 - 7
 *
 * No return
 */
extern void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

/**
 * Initialize LCD pins
 *
 * Called internally
 *
 * No return
 */
extern void TM_HD44780_InitPins(void);

/**
 * Send command to lcd
 *
 * Called internally
 *
 * No return
 */
extern void TM_HD44780_Cmd(uint8_t cmd);

/**
 * Send 4bit command to lcd
 *
 * Called internally
 *
 * No return
 */
extern void TM_HD44780_Cmd4bit(uint8_t cmd);

/**
 * Send data to lcd
 *
 * Called internally
 *
 * No return
 */
extern void TM_HD44780_Data(uint8_t data);

/**
 * Set cursor to x and y location
 *
 * Parameters:
 * 	- uint8_t col
 * 		X position
 * 	- uint8_t row
 * 		Y position
 *
 * No return
 */
extern void TM_HD44780_CursorSet(uint8_t col, uint8_t row);

#endif

