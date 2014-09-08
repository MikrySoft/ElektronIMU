/*
 * LSM9DS0_registers.h
 *
 *  Created on: Aug 29, 2014
 *      Author: mikrysoft
 */

#ifndef LSM9DS0_REGISTERS_H_
#define LSM9DS0_REGISTERS_H_



#define GYRO_ADDR 0xD6

#define LSM9DS0_GYRO_WHOAMI						0x0F	/* Device identification register = 0xD4 									*/
	#define LSM9DS0_GYRO_WHOAMI_DEFAULT			0xD4
#define LSM9DS0_GYRO_CTRL_REG1					0x20	/* Control Register 1 														*/
	#define LSM9DS0_GYRO_CTRL_REG1_DEFAULT		0x07	/* Control Register 1 Default value 														*/
	#define LSM9DS0_GYRO_CTRL_REG1_DR			0xC0	/* Data rate																*/
	#define LSM9DS0_GYRO_ODR_95HZ				0x00
	#define LSM9DS0_GYRO_ODR_190HZ				0x40
	#define LSM9DS0_GYRO_ODR_380HZ				0x80
	#define LSM9DS0_GYRO_ODR_760HZ				0xC0


	#define LSM9DS0_GYRO_CTRL_REG1_BW			0x30	/* Bandwidth 																*/
	#define LSM9DS0_GYRO_BW_LOW					0x00
	#define LSM9DS0_GYRO_BW_MID					0x10
	#define LSM9DS0_GYRO_BW_HIGH				0x20
	#define LSM9DS0_GYRO_BW_BEST				0x30


	#define LSM9DS0_GYRO_CTRL_REG1_PD			0x08	/* Power down																*
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 *	 	 0 - Power down														*
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 *	 	 1 - Normal/Sleep (when XEN&YEN&ZEN = 0)							*/
	#define LSM9DS0_GYRO_CTRL_REG1_ZEN			0x04	/* Z axis enabled															*/
	#define LSM9DS0_GYRO_CTRL_REG1_YEN			0x02	/* Y axis enabled															*/
	#define LSM9DS0_GYRO_CTRL_REG1_XEN			0x01	/* X axis enabled															*/

#define LSM9DS0_GYRO_CTRL_REG2					0x21	/* Control Register 2 														*/
	#define LSM9DS0_GYRO_CTRL_REG2_DEFAULT		0x00	/* Control Register 2 														*/
	#define LSM9DS0_GYRO_CTRL_REG2_HPM			0x30 	/* (00) High-pass filter mode:  											*
														 *	Brea		00:	Normal (reset reading HP_RESET_FILTER)					*
														 *			01: Reference for filtering										*
														 *			10: Normal														*
														 *			11: Autoreset on interrupt										*/
	#define LSM9DS0_GYRO_HPM_NORMAL_RESET		0x00
	#define LSM9DS0_GYRO_HPM_REFERENCE			0x10
	#define LSM9DS0_GYRO_HPM_NORMAL				0x20
	#define LSM9DS0_GYRO_HPM_AUTORESET			0x30

	#define LSM9DS0_GYRO_CTRL_REG2_HPCF			0x0F	/* High-pass filter cutoff (Hz):											*
														 *				ODR=	95Hz	190Hz	380Hz	760Hz						*
														 *			0000		7.2		13.5	27		51.4						*
														 *			0001		3.5		7.2		13.5	27							*
														 *			0010		1.8		3.5		7.2		13.5						*
														 *			0011		0.9		1.8		3.5		7.2							*
														 *			0100		0.45	0.9		1.8		3.5							*
														 *			0101		0.18	0.45	0.9		1.8							*
														 *			0110		0.09	0.18	0.45	0.9							*
														 *			0111		0.045	0.09	0.18	0.45						*
														 *			1000		0.018	0.045	0.09	0.18						*
														 *			1001		0.009	0.018	0.045	0.09						*/
	#define LSM9DS0_GYRO_HPCF_DIV1			0x00
	#define LSM9DS0_GYRO_HPCF_DIV2			0x01
	#define LSM9DS0_GYRO_HPCF_DIV4			0x02
	#define LSM9DS0_GYRO_HPCF_DIV8			0x03
	#define LSM9DS0_GYRO_HPCF_DIV16			0x04
	#define LSM9DS0_GYRO_HPCF_DIV32			0x05
	#define LSM9DS0_GYRO_HPCF_DIV64			0x06
	#define LSM9DS0_GYRO_HPCF_DIV128		0x07
	#define LSM9DS0_GYRO_HPCF_DIV256		0x08
	#define LSM9DS0_GYRO_HPCF_DIV512		0x09


#define LSM9DS0_GYRO_CTRL_REG3					0x22	/* Control Register 3 														*/
	#define LSM9DS0_GYRO_CTRL_REG3_DEFAULT		0x00	/* Control Register 3 														*/
	#define LSM9DS0_GYRO_CTRL_REG3_I1_INT1		0x80	/*	(0) Interrupt enable on INT_G. Active high								*/
	#define LSM9DS0_GYRO_CTRL_REG3_I1_BOOT		0x40	/*	(0) Boot status avaliable on INT_G. Active high							*/
	#define LSM9DS0_GYRO_CTRL_REG3_H_Lactive	0x20	/*	(0) Interrupt active configuration on INT_G. 0=high, 1=low				*/
	#define LSM9DS0_GYRO_CTRL_REG3_PP_OD		0x10	/*	(0) 1=Open Drain, 0 = Push-Pull											*/
	#define LSM9DS0_GYRO_CTRL_REG3_I2_DRDY		0x08	/*	(0) Data ready on DRDY_G. Active high									*/
	#define LSM9DS0_GYRO_CTRL_REG3_I2_WTM		0x04	/*	(0) FIFO watermark interrupt on DRDY_G. Active high						*/
	#define LSM9DS0_GYRO_CTRL_REG3_I2_ORun		0x02	/*	(0) FIFO overrun interrupt on DRDY_G. Active high						*/
	#define LSM9DS0_GYRO_CTRL_REG3_I2_Empty		0x01	/*	(0)	FIFO empty interrupt on DRDY_G. Active high							*/

#define LSM9DS0_GYRO_CTRL_REG4					0x23	/* Control Register 4 														*/
	#define LSM9DS0_GYRO_CTRL_REG4_DEFAULT		0x00	/* Control Register 4 														*/
	#define LSM9DS0_GYRO_CTRL_REG4_BDU			0x80	/*	(0) Block data update. 													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			0 - Continuous update,											*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			1 - output registers not updated until MSB and LSB read			*/
	#define LSM9DS0_GYRO_CTRL_REG4_BLE			0x40	/*	(0) Big/little endian 0 - LSB@lower addr, 1 - MSB@lower addr			*/
	#define LSM9DS0_GYRO_CTRL_REG4_FS			0x30	/*	(00) Full scale selection												*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 * 			00	- 245 dps													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			01	- 500 dps													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			10  - 2000 dps													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			11  - 2000 dps													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *	x:	RESERVED															*/
	#define LSM9DS0_GYRO_FS_245DPS				0x00
	#define LSM9DS0_GYRO_FS_500DPS				0x10
	#define LSM9DS0_GYRO_FS_2000DPS				0x20
	#define LSM9DS0_GYRO_FS_MULTIPLIER_245DPS		(8.75f)
	#define LSM9DS0_GYRO_FS_MULTIPLIER_500DPS		(17.5f)
	#define LSM9DS0_GYRO_FS_MULTIPLIER_2000DPS		(70.0f)

	#define LSM9DS0_GYRO_CTRL_REG4_ST			0x06	/*	(00) Self-test mode														*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			00	- normal mode												*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			01  - self test 0 (X+, Y-, Z-)									*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			10	- forbidden													*
	 	 	 	 	 	 	 	 	 	 	 	 	 	 *			11	- Self-test 1 (X-, Y+, Z+)									*/
	#define LSM9DS0_GYRO_ST_NORMAL				0x00
	#define LSM9DS0_GYRO_ST_SELF_0				0x02
	#define LSM9DS0_GYRO_ST_SELF_1				0x04

	#define LSM9DS0_GYRO_CTRL_REG4_SIM			0x01	/*	(0)	SPI mode selection. 0 - 4 wire, 1 - 3 wire	 	 	 	 	 	 	*/

#define LSM9DS0_GYRO_CTRL_REG5					0x24	/* Control Register 5 														*/
	#define LSM9DS0_GYRO_CTRL_REG5_DEFAULT		0x00	/* Control Register 5 														*/
	#define LSM9DS0_GYRO_CTRL_REG5_BOOT			0x80	/*	(0) Reboot memory content. 0 - normal mode, 1 - reboot memory			*/
	#define LSM9DS0_GYRO_CTRL_REG5_FIFO_EN		0x40	/*	(0) FIFO Enable. Active high.											*/
	#define LSM9DS0_GYRO_CTRL_REG5_H_PEN		0x20	/*	(0) High pass filter enable. Active high								*
												0x10	 *	RESERVED																*/
	#define LSM9DS0_GYRO_CTRL_REG5_INT1_Sel		0x0C	/*	(00) INT1 selection configuration										*/
	#define LSM9DS0_GYRO_INT1_SRC_LPF1			0x00
	#define LSM9DS0_GYRO_INT1_SRC_HPF			0x04
	#define LSM9DS0_GYRO_INT1_SRC_LPF2			0x08



	#define LSM9DS0_GYRO_CTRL_REG5_OUT_Sel		0x03	/*	(00) Out selection configuration										*/
	#define LSM9DS0_GYRO_OUT_SRC_LPF1			0x00
	#define LSM9DS0_GYRO_OUT_SRC_HPF			0x01
	#define LSM9DS0_GYRO_OUT_SRC_LPF2			0x02

#define LSM9DS0_GYRO_REFERENCE					0x25	/*	Reference value for interrupt generation. Default 0						*/
	#define LSM9DS0_GYRO_REFERENCE_DEFAULT		0x00	/*	Reference value for interrupt generation. Default 0						*/
#define LSM9DS0_GYRO_STATUS_REG					0x27	/* Status register															*/
	#define LSM9DS0_GYRO_STATUS_REG_ZYXOR		0x80	/*	(0)	X,Y,Z data overrun. 												*
														 *			0 - no overrun, 												*
														 *			1 - data overwritten before read								*/
	#define LSM9DS0_GYRO_STATUS_REG_ZOR			0x40	/*	(0) Z axis overrun														*/
	#define LSM9DS0_GYRO_STATUS_REG_YOR			0x20	/*	(0) Y axis overrun														*/
	#define LSM9DS0_GYRO_STATUS_REG_XOR			0x10	/*	(0)	X axis overrun														*/
	#define LSM9DS0_GYRO_STATUS_REG_ZYXDA		0x08	/*	(0)	X,Y,Z axis data available											*/
	#define LSM9DS0_GYRO_STATUS_REG_ZDA			0x04	/*	(0) Z axis data available												*/
	#define LSM9DS0_GYRO_STATUS_REG_YDA			0x02	/*	(0)	Y axis data available												*/
	#define LSM9DS0_GYRO_STATUS_REG_XDA			0x01	/*	(0) X axis data available												*/


#define LSM9DS0_GYRO_ADDR						0xA8
#define LSM9DS0_GYRO_COUNT						6
	#define LSM9DS0_GYRO_OUT_X_L				0x28	/* X axis output register LSB												*/
	#define LSM9DS0_GYRO_OUT_X_H				0x29	/* X axis output register MSB												*/
	#define LSM9DS0_GYRO_OUT_Y_L				0x2A	/* Y axis output register LSB												*/
	#define LSM9DS0_GYRO_OUT_Y_H				0x2B	/* Y axis output register MSB												*/
	#define LSM9DS0_GYRO_OUT_Z_L				0x2C	/* Z axis output register LSB												*/
	#define LSM9DS0_GYRO_OUT_Z_H				0x2D	/* Z axis output register MSB												*/

#define LSM9DS0_GYRO_FIFO_CTRL					0x2E	/* FIFO Control register													*/
	#define LSM9DS0_GYRO_FIFO_CTRL_DEFAULT		0x00	/* FIFO Control register													*/
	#define LSM9DS0_GYRO_FIFO_CTRL_FM			0xE0	/*  (000) FIFO mode selection												*
														 *			000	Bypass														*												*
														 *			001	FIFO														*
														 *			010	Stream														*
														 *			011 Stream to FIFO												*
														 *			100	Bypass to Stream											*/
	#define LSM9DS0_GYRO_FIFO_CTRL_FM_BYPASS	0x00
	#define LSM9DS0_GYRO_FIFO_CTRL_FM_FIFO		0x20
	#define LSM9DS0_GYRO_FIFO_CTRL_FM_STREAM	0x40
	#define LSM9DS0_GYRO_FIFO_CTRL_FM_STR_FIFO	0x60
	#define LSM9DS0_GYRO_FIFO_CTRL_FM_BYP_STR	0x80

	#define LSM9DS0_GYRO_FIFO_CTRL_WTM			0x1F	/*  (00000) Watermark level													*/

#define LSM9DS0_GYRO_FIFO_SRC					0x2F	/* FIFO status register														*/
	#define LSM9DS0_GYRO_FIFO_SRC_WTM			0x80	/*	Watermark status
																	0 - FIFO lower than watermark,
																	1 - FIFO at or above watermark 									*/
	#define LSM9DS0_GYRO_FIFO_SRC_OVRN			0x40	/*	FIFO completely filled													*/
	#define LSM9DS0_GYRO_FIFO_SRC_EMPTY			0x20	/*	FIFO empty																*/
	#define LSM9DS0_GYRO_FIFO_SRC_FSS			0x1F	/*	FIFO stored data level													*/
#define LSM9DS0_GYRO_INT1_CFG					0x30	/* Interrupt 1 configuration register										*/
	#define LSM9DS0_GYRO_INT1_CFG_DEFAULT		0x00	/* Interrupt 1 configuration register										*/
	#define LSM9DS0_GYRO_INT1_CFG_AOR			0x80	/*	(0)	AND/OR combination of interrupt events. 0 - OR, 1 - AND				*/
	#define LSM9DS0_GYRO_INT1_CFG_LIR			0x40	/*	(0)	Latch interrupt request.
																	0 - interrupt request not latched,
																	1 - interrupt request latched									*/
														/*	Cleared by reading INT1_SRC												*/
	#define LSM9DS0_GYRO_INT1_CFG_ZHIE			0x20	/*	(0)	Enable interrupt on Z high event									*/
	#define LSM9DS0_GYRO_INT1_CFG_ZLIE			0x10	/* 	(0)	Enable interrupt on Z low event										*/
	#define LSM9DS0_GYRO_INT1_CFG_YHIE			0x08	/*	(0)	Enable interrupt on Y high event									*/
	#define LSM9DS0_GYRO_INT1_CFG_YLIE			0x04	/* 	(0)	Enable interrupt on Y low event										*/
	#define LSM9DS0_GYRO_INT1_CFG_XHIE			0x02	/*	(0)	Enable interrupt on X high event									*/
	#define LSM9DS0_GYRO_INT1_CFG_XLIE			0x01	/* 	(0)	Enable interrupt on X low event										*/

#define LSM9DS0_GYRO_INT1_SRC					0x31	/* Interrupt 1 source register (Read Only)									*/
	#define LSM9DS0_GYRO_INT1_SRC_IA			0x40	/*			(0) Interrupt active 1 - one or more interrupts happened		*/
	#define LSM9DS0_GYRO_INT1_SRC_ZH			0x20	/*			(0)	Z high event occurred										*/
	#define LSM9DS0_GYRO_INT1_SRC_ZL			0x10	/*			(0)	Z low event occurred										*/
	#define LSM9DS0_GYRO_INT1_SRC_YH			0x08	/*			(0)	Y high event occurred										*/
	#define LSM9DS0_GYRO_INT1_SRC_YL			0x04	/*			(0)	Y low event occurred										*/
	#define LSM9DS0_GYRO_INT1_SRC_XH			0x02	/*			(0)	X high event occurred										*/
	#define LSM9DS0_GYRO_INT1_SRC_XL			0x01	/*			(0)	X low event occurred										*/

#define LSM9DS0_GYRO_INT1_TH_ADDR				0xB2
#define LSM9DS0_GYRO_INT1_TH_COUNT				6
	#define LSM9DS0_GYRO_INT1_TH_XH				0x32	/* Interrupt 1 Threshold: X MSB	Max: 0x7F									*/
	#define LSM9DS0_GYRO_INT1_TH_XL				0x33	/* Interrupt 1 Threshold: X LSB												*/
	#define LSM9DS0_GYRO_INT1_TH_YH				0x34	/* Interrupt 1 Threshold: Y MSB	Max: 0x7F 									*/
	#define LSM9DS0_GYRO_INT1_TH_YL				0x35	/* Interrupt 1 Threshold: Y LSB												*/
	#define LSM9DS0_GYRO_INT1_TH_ZH				0x36	/* Interrupt 1 Threshold: X MSB	Max: 0x7F 									*/
	#define LSM9DS0_GYRO_INT1_TH_ZL				0x37	/* Interrupt 1 Threshold: X LSB												*/

#define LSM9DS0_GYRO_INT1_DUR					0x38	/* Interrupt 1 configuration register 										*/
	#define LSM9DS0_GYRO_INT1_DUR_WAIT			0x80	/*	Wait enable. Active high
																	0: Interrupt disappears the moment threshold is crossed
																	1: Interrupt disappears if threshold is crossed for more than D samples */
	#define LSM9DS0_GYRO_INT1_DUR_D				0x7F	/*	Duration value 															*/

/*****************************************************************************************************************************************************/
#define ACCEL_ADDR 0x3A


#define LSM9DS0_TEMP_ADDR						0x85
#define LSM9DS0_TEMP_COUNT						2
	#define LSM9DS0_TEMP_L						0x05	/* Temperature sensor LSB */
	#define LSM9DS0_TEMP_H						0x06	/* Temperature sensor MSB */

#define LSM9DS0_MAG_STATUS						0x07	/* Magnetometer status register */
	#define LSM9DS0_MAG_STATUS_ZYXMOR			0x80	/*		X,Y,Z data overrun	*/
	#define LSM9DS0_MAG_STATUS_ZMOR				0x40	/*		Z data overrun	*/
	#define LSM9DS0_MAG_STATUS_YMOR				0x20	/*		Y data overrun	*/
	#define LSM9DS0_MAG_STATUS_XMOR				0x10	/*		X data overrun	*/
	#define LSM9DS0_MAG_STATUS_ZYXMDA			0x08	/*		X,Y,Z new data available	*/
	#define LSM9DS0_MAG_STATUS_ZMDA				0x04	/*		Z new data available	*/
	#define LSM9DS0_MAG_STATUS_YMDA				0x02	/*		Y new data available	*/
	#define LSM9DS0_MAG_STATUS_XMDA				0x01	/*		X new data available	*/

#define LSM9DS0_MAG_ADDR						0x88
#define LSM9DS0_MAG_COUNT						6
	#define LSM9DS0_MAG_OUT_X_L					0x08	/*  X axis magnetic data LSB */
	#define LSM9DS0_MAG_OUT_X_H					0x09	/*  X axis magnetic data MSB		16 bit, two's complement, left justified */
	#define LSM9DS0_MAG_OUT_Y_L					0x0A	/*  Y axis magnetic data LSB */
	#define LSM9DS0_MAG_OUT_Y_H					0x0B	/*  Y axis magnetic data MSB */
	#define LSM9DS0_MAG_OUT_Z_L					0x0C	/*  Z axis magnetic data LSB */
	#define LSM9DS0_MAG_OUT_Z_H					0x0D	/*  Z axis magnetic data MSB */

#define LSM9DS0_MAG_WHOAMI						0x0F	/*  Device identification register = 0x49 */
	#define LSM9DS0_MAG_WHOAMI_DEFAULT			0x49	/*  Device identification register = 0x49 */

#define LSM9DS0_MAG_INT_CFG						0x12	/* Interrupt configuration register */
	#define LSM9DS0_MAG_INT_CFG_DEFAULT			0xE8	/* Interrupt configuration register */
	#define LSM9DS0_MAG_INT_CFG_XMIEN			0x80	/*		(0) Enable interrupt on X axis	*/
	#define LSM9DS0_MAG_INT_CFG_YMIEN			0x40	/*		(0) Enable interrupt on Y axis	*/
	#define LSM9DS0_MAG_INT_CFG_ZMIEN			0x20	/*		(0) Enable interrupt on Z axis	*/
	#define LSM9DS0_MAG_INT_CFG_PP_OD			0x10	/*		(0)	Interrupt pin configuration 0 - push-pull, 1 - open drain	*/
	#define LSM9DS0_MAG_INT_CFG_IEA				0x08	/*		(0)	Interrupt polarity for accelerometer and magnetometer 0 - active low, 1 - active high	*/
	#define LSM9DS0_MAG_INT_CFG_IEL				0x04	/*		(0) Latch interrupt request on INT_GEN1_SRC, INT_GEN2_SRC and INT_SRC_REG_M	*/
	#define LSM9DS0_MAG_INT_CFG_4D				0x02	/*			4D detection on acceleration data is enabled when 6D bit in INT_GEN1_REG is set to 1	*/
	#define LSM9DS0_MAG_INT_CFG_MIEN			0x01	/*		(0)	Enable interrupt generation for magnetic data	*/

#define LSM9DS0_MAG_INT_SRC						0x13	/* Interrupt source register */
	#define LSM9DS0_MAG_INT_SRC_M_PTH_X	 		0x80	/*		(0)	Magnetic value on X-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XM_PTH_Y	0x40	/*		(0)	Magnetic value on Y-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XM_PTH_Z	0x20	/*		(0)	Magnetic value on Z-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XM_NTH_X	0x10	/*		(0)	Magnetic value on X-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XM_NTH_Y	0x08	/*		(0)	Magnetic value on Y-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XM_NTH_Z	0x04	/*		(0)	Magnetic value on Z-axis exceeds the threshold on the positive side	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XMROI	 	0x02	/*			(0) Internal measurement range overflow on magnetic value. To enable set MIEN@MAG_INT_CFG	*/
	#define LSM9DS0_MAG_INT_SRC_M_PTH_XMINT	 	0x01	/*			(0)	Magnetic value over threshold	*/


#define LSM9DS0_MAG_INT_TH_ADDR					0x94
#define LSM9DS0_MAG_INT_TH_COUNT				2
	#define LSM9DS0_MAG_INT_TH_L				0x14	/* Magnetic interrupt threshold LSB. Default 0. 16bit unsigned*/
	#define LSM9DS0_MAG_INT_TH_H				0x15	/* Magnetic interrupt threshold MSB */

#define LSM9DS0_MAG_OFFSET_ADDR					0x96
#define LSM9DS0_MAG_OFFSET_COUNT				6
	#define LSM9DS0_MAG_OFFSET_X_L				0x16	/* Magnetic offset for X axis LSB. 16 bit two's complement left justfified	 */
	#define LSM9DS0_MAG_OFFSET_X_H				0x17	/* Magnetic offset for X axis MSB */
	#define LSM9DS0_MAG_OFFSET_Y_L				0x18	/* Magnetic offset for Y axis LSB. 16 bit two's complement left justfified	 */
	#define LSM9DS0_MAG_OFFSET_Y_H				0x19	/* Magnetic offset for Y axis MSB */
	#define LSM9DS0_MAG_OFFSET_Z_L				0x1A	/* Magnetic offset for Z axis LSB. 16 bit two's complement left justfified	 */
	#define LSM9DS0_MAG_OFFSET_Z_H				0x1B	/* Magnetic offset for Z axis MSB */

#define LSM9DS0_ACC_REF_ADDR					0x9C
#define LSM9DS0_ACC_REF_COUNT					3
	#define LSM9DS0_ACC_REF_X					0x1C	/* Reference value for high-pass filter for X-axis acceleration data  */
	#define LSM9DS0_ACC_REF_Y					0x1D	/* Reference value for high-pass filter for Y-axis acceleration data */
	#define LSM9DS0_ACC_REF_Z					0x1E	/* Reference value for high-pass filter for Z-axis acceleration data */

#define LSM9DS0_ACC_CTRL_REG0					0x1F	/* Accelerometer Control register 0 																	*/
	#define LSM9DS0_ACC_CTRL_REG0_DEFAULT		0x00	/* Accelerometer Control register 0 																	*/
	#define LSM9DS0_ACC_CTRL_REG0_BOOT	 		0x80	/*		(0)	Reboot memory content. 0 - normal mode, 1 - reboot memory									*/
	#define LSM9DS0_ACC_CTRL_REG0_FIFO_EN	 	0x40	/*		(0)	FIFO enable. Active high																	*/
	#define LSM9DS0_ACC_CTRL_REG0_WTM_EN	 	0x20	/*		(0)	FIFO watermark enable
												0x10			(0) RESERVED
												0x08			(0) RESERVED																					*/
	#define LSM9DS0_ACC_CTRL_REG0_HP_Click	 	0x04	/*		(0) High-pass filter enabled for Click function. 0 - filter bypassed, 1 - filter enabled		*/
	#define LSM9DS0_ACC_CTRL_REG0_HPIS1	 		0x02	/*		(0) High-pass fitler enabled for Interrupt generator 1. 0 - filter bypassed, 1 - filter enabled	*/
	#define LSM9DS0_ACC_CTRL_REG0_HIPS2	 		0x01	/*		(0) High-pass fitler enabled for Interrupt generator 2. 0 - filter bypassed, 1 - filter enabled	*/

#define LSM9DS0_ACC_CTRL_REG1					0x20	/* Accelerometer Control register 1																		*/
	#define LSM9DS0_ACC_CTRL_REG1_DEFAULT		0x07	/* Accelerometer Control register 1																		*/
	#define LSM9DS0_ACC_CTRL_REG1_AODR	 		0xF0	/*		(0000)	Acceleration data rate selection														*
																	0000	Power down mode			0001	3.125Hz												*
																	0010	6.25Hz					0011	12.5Hz												*
																	0100	25Hz					0101	50Hz												*
																	0110	100Hz					0111	200Hz												*
																	1000	400Hz					1001	800Hz												*
																	1010	1600Hz																				*/
	#define LSM9DS0_ACC_AODR_POWERDOWN			0x00
	#define LSM9DS0_ACC_AODR_3HZ				0x10
	#define LSM9DS0_ACC_AODR_6HZ				0x20
	#define LSM9DS0_ACC_AODR_12HZ				0x30
	#define LSM9DS0_ACC_AODR_25HZ				0x40
	#define LSM9DS0_ACC_AODR_50HZ				0x50
	#define LSM9DS0_ACC_AODR_100HZ				0x60
	#define LSM9DS0_ACC_AODR_200HZ				0x70
	#define LSM9DS0_ACC_AODR_400HZ				0x80
	#define LSM9DS0_ACC_AODR_800HZ				0x90
	#define LSM9DS0_ACC_AODR_1600HZ				0xA0



	#define LSM9DS0_ACC_CTRL_REG1_BDU	 		0x08	/*		(0)	Block data update for acceleration and magnetic data										*
																	0 - continuous update																		*
																	1 - output registers not updated until MSB and LSB have bean read							*/
	#define LSM9DS0_ACC_CTRL_REG1_AZEN	 		0x04	/*		(1)	Acceleration Z axis enable																	*/
	#define LSM9DS0_ACC_CTRL_REG1_AYEN	 		0x02	/*		(1)	Acceleration Y-axis enable																	*/
	#define LSM9DS0_ACC_CTRL_REG1_AXEN	 		0x01	/*		(1) Acceleration X-axis enable																	*/

#define LSM9DS0_ACC_CTRL_REG2					0x21	/* Accelerometer Control register 2 																	*/
	#define LSM9DS0_ACC_CTRL_REG2_DEFAULT		0x00	/* Accelerometer Control register 2 																	*/
	#define LSM9DS0_ACC_CTRL_REG2_ABW	 		0xC0	/*		(00)  Accelerometer anti-alias filter bandwidth													*
																	00	733Hz																					*
																	01	194Hz																					*
																	10	362Hz																					*
																	11	50Hz																					*/
	#define LSM9DS0_ACC_ANTI_ALIAS_BW_733HZ		0x00
	#define LSM9DS0_ACC_ANTI_ALIAS_BW_194HZ		0x40
	#define LSM9DS0_ACC_ANTI_ALIAS_BW_362HZ		0x80
	#define LSM9DS0_ACC_ANTI_ALIAS_BW_50HZ		0xC0


	#define LSM9DS0_ACC_CTRL_REG2_AFS	 		0x38	/*		(000) Acceleration full-scale selection														*
																	000	2g																						*
																	001	4g																						*
																	010	6g																						*
																	011	8g																						*
																	100	16g																						*/
	#define LSM9DS0_ACC_FULL_SCALE_2G			0x00
	#define LSM9DS0_ACC_FULL_SCALE_4G			0x08
	#define LSM9DS0_ACC_FULL_SCALE_6G			0x10
	#define LSM9DS0_ACC_FULL_SCALE_8G			0x18
	#define LSM9DS0_ACC_FULL_SCALE_16G			0x20

	#define LSM9DS0_ACC_MULTIPLIER_2G			(0.061f)
	#define LSM9DS0_ACC_MULTIPLIER_4G			(0.122f)
	#define LSM9DS0_ACC_MULTIPLIER_6G			(0.183f)
	#define LSM9DS0_ACC_MULTIPLIER_8G			(0.244f)
	#define LSM9DS0_ACC_MULTIPLIER_16G			(0.732f)


	#define LSM9DS0_ACC_CTRL_REG2_AST	 		0x06	/*		(00) Acceleration self-test enable																*
																	00	Normal mode																				*
																	01	Positive sign self-test																	*
																	10	Negative sign self-test																	*
																	11	Not allowed																				*/
	#define LSM9DS0_ACC_SELF_TEST_NORMAL		0x00
	#define LSM9DS0_ACC_SELF_TEST_POSITIVE		0x02
	#define LSM9DS0_ACC_SELF_TEST_NEGATIVE		0x04

	#define LSM9DS0_ACC_CTRL_REG2_SIM	 		0x01	/*		(0) SPI Serial Interface Mode selection															*/

#define LSM9DS0_MAG_CTRL_REG3					0x22	/* Accelerometer Control register 3 */
	#define LSM9DS0_MAG_CTRL_REG3_DEFAULT		0x00	/* Accelerometer Control register 3 */
	#define LSM9DS0_MAG_CTRL_REG3_P1_BOOT	 	0x80	/*		(0)	Boot on INT1_XM enable	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_TAP	 	0x40	/*		(0)	Tap generator interrupt on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_INT1	 	0x20	/*		(0)	Inertial interrupt generator 1 on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_INT2	 	0x10	/*		(0) Inertial interrupt generator 2 on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_INTM	 	0x08	/*		(0)	Magnetic interrupt generator on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_DRDYA		0x04	/*		(0)	Accelerometer data-ready signal on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_DRDYM	 	0x02	/*		(0) Magnetometer data-ready signal on INT1_XM	*/
	#define LSM9DS0_MAG_CTRL_REG3_P1_EMPTY	 	0x01	/*		(0) FIFO empty indication on INT1_XM	*/


#define LSM9DS0_MAG_CTRL_REG4					0x23	/* Accelerometer Control register 4 */
	#define LSM9DS0_MAG_CTRL_REG_DEFAULT		0x00	/* Accelerometer Control register 4 */
	#define LSM9DS0_MAG_CTRL_REG4_P2_TAP	 	0x80	/*		(0)	Tap generator interrupt on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_INT1	 	0x40	/*		(0)	Inertial interrupt generator 1 on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_INT2	 	0x20	/*		(0) Inertial interrupt generator 2 on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_INTM	 	0x10	/*		(0)	Magnemoic interrupt generator on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_DRDYA	 	0x08	/*		(0)	Accelerometer data-ready signal on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_DRDYM	 	0x04	/*		(0) Magnetometer data-ready signal on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_EMPTY	 	0x02	/*		(0) FIFO empty indication on INT2_XM	*/
	#define LSM9DS0_MAG_CTRL_REG4_P2_WTM		0x01	/*		(0) FIFO watermark on INT2_XM	*/

#define LSM9DS0_MAG_CTRL_REG5					0x24	/* Accelerometer Control register 5 */
	#define LSM9DS0_MAG_CTRL_REG5_DEFAULT		0x18	/* Accelerometer Control register 5 */
	#define LSM9DS0_MAG_CTRL_REG5_TEMP_EN		0x80	/*		(0)	Temperature sensor enable	*/
	#define LSM9DS0_MAG_CTRL_REG5_M_RES	 		0x60	/*		(00) Magnetic resolution selection: 00 - low resolution, 11 - high resolution	*/
	#define LSM9DS0_MAG_RESOLUTION_LOW			0x00
	#define LSM9DS0_MAG_RESOLUTION_HIGH			0x60

	#define LSM9DS0_MAG_CTRL_REG5_M_ODR	 		0x1C	/*		(110) Magnetic data rate selection
																	000	3.125Hz
																	001	6.25Hz
																	010	12.5Hz
																	011	25Hz
																	100	50Hz
																	101	100Hz (requires accelerometer ODR > 50Hz or power-down)
																	110	Reserved
																	111	Reserved	*/
	#define LSM9DS0_MAG_DATA_RATE_3HZ			0x00
	#define LSM9DS0_MAG_DATA_RATE_6HZ			0x04
	#define LSM9DS0_MAG_DATA_RATE_12HZ			0x08
	#define LSM9DS0_MAG_DATA_RATE_25HZ			0x0C
	#define LSM9DS0_MAG_DATA_RATE_50HZ			0x10
	#define LSM9DS0_MAG_DATA_RATE_100HZ			0x14


	#define LSM9DS0_MAG_CTRL_REG5_LIR2	 		0x02	/*			(0) Latch interrupt request on INT2_SRC with it being cleared by reading INT2_SRC itself	*/
	#define LSM9DS0_MAG_CTRL_REG5_LIR1	 		0x01	/*			(0) Latch interrupt request on INT1_SRC with it being cleared by reading INT1_SRC itself	*/

#define LSM9DS0_MAG_CTRL_REG6					0x25	/* Accelerometer Control register 6 */
	#define LSM9DS0_MAG_CTRL_REG6_DEFAULT		0x20	/* Accelerometer Control register 6
												0x80			(0) Reserved	*/
	#define LSM9DS0_MAG_CTRL_REG6_MFS			0x60	/*		(01) Magnetic full-scale selection
																	00	2 gauss			01	4 gauss
																	10	8 gauss			11	12 gauss	*/
	#define LSM9DS0_MAG_FULL_SCALE_2G			0x00
	#define LSM9DS0_MAG_FULL_SCALE_4G			0x20
	#define LSM9DS0_MAG_FULL_SCALE_8G			0x40
	#define LSM9DS0_MAG_FULL_SCALE_12G			0x60
	#define LSM9DS0_MAG_MULTIPLIER_2G			(0.08f)
	#define LSM9DS0_MAG_MULTIPLIER_4G			(0.16f)
	#define LSM9DS0_MAG_MULTIPLIER_8G			(0.32f)
	#define LSM9DS0_MAG_MULTIPLIER_12G			(0.48f)


#define LSM9DS0_MAG_CTRL_REG7					0x26	/* Accelerometer Control register 7 */
	#define LSM9DS0_MAG_CTRL_REG7_DEFAULT		0x03	/* Accelerometer Control register 7 */
	#define LSM9DS0_MAG_CTRL_REG7_AHPM	 		0xC0	/*	 	(00) High-pass filter mode selection for acceleration data
																	00	Normal mode (resets X,Y,Z axis reading REFERENCE_X,Y,Z)
																	01	Reference signal for filtering
																	10	Normal mode
																	11	Autoreset on interrupt	*/

	#define LSM9DS0_MAG_HPM_NORMAL_RESET		0x00
	#define LSM9DS0_MAG_HPM_NORMAL_REFERENCE	0x40
	#define LSM9DS0_MAG_HPM_NORMAL_NORMAL		0x80
	#define LSM9DS0_MAG_HPM_NORMAL_AUTORESET	0xC0

	#define LSM9DS0_MAG_CTRL_REG7_AFDS	 		0x20	/*		(0)	Filtered acceleration data selection.
																	0 - filter bypassed
																	1 - data from internal filter sent to output register and FIFO
												0x18 			(00) RESERVED	*/
	#define LSM9DS0_MAG_CTRL_REG7_MLP			0x04	/*		(0)	Magnetic data low-power mode. 1 overrides MODR to 3.125Hz	*/
	#define LSM9DS0_MAG_CTRL_REG7_MD	 		0x03	/*		(10) Magnetic sensor mode selection
																	00	Continuous-conversion mode
																	01	Single-conversion mode
																	10	Power down mode
																	11	Power down mode	*/
	#define LSM9DS0_MAG_MODE_CONTINUOUS			0x00
	#define LSM9DS0_MAG_MODE_SINGLE				0x01
	#define LSM9DS0_MAG_MODE_POWER_DOWN			0x02

#define LSM9DS0_ACC_STATUS						0x27	/* Accelerometer status register */
	#define LSM9DS0_ACC_STATUS_ZYXAOR	 		0x80	/*		(0) Acceleration X,Y,Z axis data overrun	*/
	#define LSM9DS0_ACC_STATUS_ZAOR	 			0x40	/*		(0)	Acceleration Z axis overrun	*/
	#define LSM9DS0_ACC_STATUS_YAOR	 			0x20	/*		(0)	Acceleration Y axis overrun	*/
	#define LSM9DS0_ACC_STATUS_XAOR	 			0x10	/*		(0)	Acceleration X axis overrun	*/
	#define LSM9DS0_ACC_STATUS_ZYXADA	 		0x08	/*		(0)	Acceleration X,Y,Z axis new value available	*/
	#define LSM9DS0_ACC_STATUS_ZADA	 			0x04	/*		(0)	Acceleration Z axis new value available	*/
	#define LSM9DS0_ACC_STATUS_YADA	 			0x02	/*		(0) Acceleration Y axis new value available	*/
	#define LSM9DS0_ACC_STATUS_XADA	 			0x01	/*		(0)	Acceleration X axis new value available	*/


#define LSM9DS0_ACC_OUT_ADDR					0xA8
#define LSM9DS0_ACC_OUT_COUNT					6
	#define LSM9DS0_ACC_OUT_X_L					0x28	/*  X axis acceleration data LSB. 16 bit, two's complement left justified */
	#define LSM9DS0_ACC_OUT_X_H					0x29	/*  X axis acceleration data MSB */
	#define LSM9DS0_ACC_OUT_Y_L					0x2A	/*  Y axis acceleration data LSB */
	#define LSM9DS0_ACC_OUT_Y_H					0x2B	/*  Y axis acceleration data MSB */
	#define LSM9DS0_ACC_OUT_Z_L					0x2C	/*  Z axis acceleration data LSB */
	#define LSM9DS0_ACC_OUT_Z_H					0x2D	/*  Z axis acceleration data MSB */

#define LSM9DS0_ACC_FIFO_CFG					0x2E	/* FIFO Configuration register */
	#define LSM9DS0_ACC_FIFO_CFG_FM				0xE0	/*		(000) FIFO mode selection
																	000	Bypass mode
																	001	FIFO mode
																	010	Stream mode
																	011	Stream to FIFO mode
																	100	Bypass to stream mode	*/

	#define LSM9DS0_ACC_FIFO_CTRL_FM_BYPASS		0x00
	#define LSM9DS0_ACC_FIFO_CTRL_FM_FIFO		0x20
	#define LSM9DS0_ACC_FIFO_CTRL_FM_STREAM		0x40
	#define LSM9DS0_ACC_FIFO_CTRL_FM_STR_FIFO	0x60
	#define LSM9DS0_ACC_FIFO_CTRL_FM_BYP_STR	0x80

	#define LSM9DS0_ACC_FIFO_CFG_FTH	 		0x1F	/*		(0 0000) FIFO watermark level	*/

#define LSM9DS0_ACC_FIFO_SRC					0x2F	/* FIFO status register */
	#define LSM9DS0_ACC_FIFO_SRC_WTM	 		0x80	/*			Watermark status 1 - FIFO level over watermark	*/
	#define LSM9DS0_ACC_FIFO_SRC_OVRN	 		0x40	/*			FIFO Overrun status	*/
	#define LSM9DS0_ACC_FIFO_SRC_EMPTY	 		0x20	/*			FIFO Empty status	*/
	#define LSM9DS0_ACC_FIFO_SRC_FSS	 		0x1F	/*	 	FIFO stored data level - current number of unread FIFO levels	*/


#define LSM9DS0_ACC_INT1_GEN					0x30	/* Interrupt generator 1 control register */
	#define LSM9DS0_ACC_INT1_GEN_AOI	 		0x80	/*		(0)	And/Or combination of interrupts	*/
	#define LSM9DS0_ACC_INT1_GEN_6D	 			0x40	/*			(0)	5-direction detection function enabled
																	AOI 6D
																	  0 0		OR combination of events
																	  0 1		6D movement recognition - interrupt triggered when orientation comes into known zone, stays for ODR
																	  1 0		AND combination of interrupt events
																	  1 1		6D position recognition - interrupt triggered when orientation is inside known zone, stays active for the whole time */
	#define LSM9DS0_ACC_INT1_GEN_ZHIE_ZUPE		0x20	 /*		(0)	Enable interrupt generation on Z high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT1_GEN_ZLIE_ZDOWNE	0x10	 /*		(0)	Enable interrupt generation on Z low event or on direction recognition	*/
	#define LSM9DS0_ACC_INT1_GEN_YHIE_YUPE		0x08	 /*		(0)	Enable interrupt generation on Y high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT1_GEN_YLIE_YDOWNE	0x04	 /*		(0) Enable interrupt generation on Y low event or on direction recognition	*/
	#define LSM9DS0_ACC_INT1_GEN_XHIE_XUPE		0x02	 /*		(0)	Enable interrupt generation on X high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT1_GEN_XLIE_XDOWNE	0x01	 /*		(0) Enable interrupt generation on X low event or on direction recognition	*/

#define LSM9DS0_ACC_INT1_SRC					0x31	/* Interrupt 1 status register
 	 	 	 	 	 	 	 	 				0x80			(0)	RESERVED	*/
	#define LSM9DS0_ACC_INT1_SRC_IA	 			0x40	/*		(0)	Interrupt status	*/
	#define LSM9DS0_ACC_INT1_SRC_ZH				0x20	/*		(0)	Z high	*/
	#define LSM9DS0_ACC_INT1_SRC_ZL	 			0x10	/*		(0)	Z low	*/
	#define LSM9DS0_ACC_INT1_SRC_YH	 			0x08	/*		(0)	Y high	*/
	#define LSM9DS0_ACC_INT1_SRC_YL	 			0x04	/*		(0)	Y low	*/
	#define LSM9DS0_ACC_INT1_SRC_XH	 			0x02	/*		(0)	X high	*/
	#define LSM9DS0_ACC_INT1_SRC_XL	 			0x01	/*		(0)	X low	*/

#define LSM9DS0_ACC_INT1_THS					0x32	/* Interrupt 1 threshold  = 0x00-	0x7F */
#define LSM9DS0_ACC_INT1_DUR					0x33	/* Interrupt 1 duration= 0x00-	0x7F, steps depend on ODR  */
#define LSM9DS0_ACC_INT2_GEN					0x34	/* Interrupt 2 configuration register */
	#define LSM9DS0_ACC_INT2_GEN_AOI			0x80	/*		(0)	And/Or combination of interrupts	*/
	#define LSM9DS0_ACC_INT2_GEN_6D				0x40	/*		(0)	5-direction detection function enabled
																	AOI 6D
																	0 0		OR combination of events
																	0 1		6D movement recognition - interrupt triggered when orientation comes into known zone, stays for ODR
																	1 0		AND combination of interrupt events
																	1 1		6D position recognition - interrupt triggered when orientation is inside known zone, stays active for the whole time*/
	#define LSM9DS0_ACC_INT2_GEN_ZHIE_ZUPE		0x20	/*		(0)	Enable interrupt generation on Z high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT2_GEN_ZLIE_ZDOWNE	0x10	/*		(0)	Enable interrupt generation on Z low event or on direction recognition	*/
	#define LSM9DS0_ACC_INT2_GEN_YHIE_YUPE		0x08	/*		(0)	Enable interrupt generation on Y high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT2_GEN_YLIE_YDOWNE	0x04	/*		(0) Enable interrupt generation on Y low event or on direction recognition	*/
	#define LSM9DS0_ACC_INT2_GEN_XHIE_XUPE		0x02	/*		(0)	Enable interrupt generation on X high event or on direction recognition	*/
	#define LSM9DS0_ACC_INT2_GEN_XLIE_XDOWNE	0x01	/*		(0) Enable interrupt generation on X low event or on direction recognition	*/

#define LSM9DS0_ACC_INT2_SRC					0x35	/* Interrupt 2 status register
												0x80			(0)	RESERVED */
	#define LSM9DS0_ACC_INT2_SRC_IA	 			0x40	/*		(0)	Interrupt status	*/
	#define LSM9DS0_ACC_INT2_SRC_ZH	 			0x20	/*		(0)	Z high	*/
	#define LSM9DS0_ACC_INT2_SRC_ZL				0x10	/*		(0)	Z low	*/
	#define LSM9DS0_ACC_INT2_SRC_YH	 			0x08	/*		(0)	Y high	*/
	#define LSM9DS0_ACC_INT2_SRC_YL	 			0x04	/*		(0)	Y low	*/
	#define LSM9DS0_ACC_INT2_SRC_XH	 			0x02	/*		(0)	X high	*/
	#define LSM9DS0_ACC_INT2_SRC_XL	 			0x01	/*		(0)	X low	*/

#define LSM9DS0_ACC_INT2_THS					0x36	/*  Interrupt 2 threshold  = 0x00-	0x7F */
#define LSM9DS0_ACC_INT2_DUR					0x37	/*  Interrupt 2 duration= 0x00-	0x7F, steps depend on ODR  */
#define LSM9DS0_ACC_CLICK_CFG					0x38	/* Click detection configuration
												0xC0			(xx)	RESERVED	*/
	#define LSM9DS0_ACC_CLICK_CFG_ZD			0x20	/*		(0)	Enable interrupt double-click on Z axis	*/
	#define LSM9DS0_ACC_CLICK_CFG_ZS			0x10	/*		(0) Enable interrpt single-click on Z axis	*/
	#define LSM9DS0_ACC_CLICK_CFG_YD			0x08	/*		(0) Enable interrupt double-click on Y axis	*/
	#define LSM9DS0_ACC_CLICK_CFG_YS			0x04	/*		(0) Enable interrupt single-click on Y axis	*/
	#define LSM9DS0_ACC_CLICK_CFG_XD			0x02	/*		(0) Enable interrupt double-click on X axis	*/
	#define LSM9DS0_ACC_CLICK_CFG_XS			0x01	/*		(0) Enable interrupt single-click on X axis	*/

#define LSM9DS0_ACC_CLICK_SRC					0x39	/* Click detection - source
												0x80			(x)	RESERVED	*/
	#define LSM9DS0_ACC_CLICK_SRC_IA	 		0x40	/*		(0)	Interrupt active	*/
	#define LSM9DS0_ACC_CLICK_SRC_DClick	 	0x20	/*		(0)	Double click detection enable	*/
	#define LSM9DS0_ACC_CLICK_SRC_SClick	 	0x10	/*		(0)	Single click detection enable	*/
	#define LSM9DS0_ACC_CLICK_SRC_Sign	 		0x08	/*		Click sign 0 - positive, 1 - negative	*/
	#define LSM9DS0_ACC_CLICK_SRC_Z				0x04	/*		(0)	Z Click detected	*/
	#define LSM9DS0_ACC_CLICK_SRC_Y				0x02	/*		(0)	Y Click detected	*/
	#define LSM9DS0_ACC_CLICK_SRC_X				0x01	/*		(0)	X Click detected	*/


#define LSM9DS0_ACC_CLICK_THS					0x3A	/*  Click threshold = 0x00-	0x7E */
#define LSM9DS0_ACC_TIME_LIMIT					0x3B	/*  Time limit = 0x00-	0x7E */
#define LSM9DS0_ACC_TIME_LATENCY				0x3C	/*  Click time latency = 0x00-	0xFF */
#define LSM9DS0_ACC_TIME_WINDOW					0x3D	/*  Click time window = 0x00-	0xFF */
#define LSM9DS0_ACC_ACT_THS						0x3E	/*  Sleep-to-Wake, Return-to-Sleep activation threshold = 0x00-	0x7F. 1LSb = 16mg */
#define LSM9DS0_ACC_ACT_DUR						0x3F	/*  Sleep-to-Wake, Return-to-Sleep duration	= 0x00-	0xFF, DUR = (ACT_DUR + 1)*8/ODR */






#endif /* LSM9DS0_REGISTERS_H_ */
