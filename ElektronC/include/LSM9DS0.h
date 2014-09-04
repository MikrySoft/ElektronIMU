/*
 * LSM9DS0.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */

#ifndef LSM9DS0_H_
#define LSM9DS0_H_

#include "LSM9DS0_registers.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"

typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
} int16_3D_t;

typedef struct {
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
} uint16_3D_t;

typedef struct {
	float X;
	float Y;
	float Z;
} float_3D_t;

typedef struct {
	I2C_TypeDef* I2C;
	uint8_t Address;
	uint8_t	DataRate;
	uint8_t Bandwidth;
	uint8_t FullScale;
	uint8_t Enabled				:1;
	uint8_t	XAxisEnabled		:1;
	uint8_t YAxisEnabled		:1;
	uint8_t	ZAxisEnabled		:1;
	uint8_t	HighPassEnable		:1;
	uint8_t HighPassMode;
	uint8_t HighPassCutoff		:4;
	uint8_t InterruptEnabled	:1;
	uint8_t	InterruptOnBoot		:1;
	uint8_t	InterruptPolarity	:1;
	uint8_t	InterruptPP_OD		:1;
	uint8_t	DRDYOnDRDY			:1;
	uint8_t	FIFOWatermarkOnDRDY	:1;
	uint8_t FIFOOverrunOnDRDY	:1;
	uint8_t	FIFOEmptyOnDRDY		:1;
	uint8_t	BlockUpdate			:1;
	uint8_t BigLittleEndian		:1;
	uint8_t FIFOEnable			:1;
	uint8_t	FIFOMode;
	uint8_t FIFOWatermark		:4;
	uint8_t	IntSource;
	uint8_t OutSource;
	uint8_t IntAndOrMode		:1;
	uint8_t IntLatching			:1;
	uint8_t IntZHighEnable		:1;
	uint8_t IntZLowEnable		:1;
	uint8_t IntYHighEnable		:1;
	uint8_t IntYLowEnable		:1;
	uint8_t IntXHighEnable		:1;
	uint8_t IntXLowEnable		:1;
} LSM9DS0_Gyro_TypeDef;


typedef struct {
	uint8_t	AndOrMode			:1;
	uint8_t	Detect6DMode		:1;
	uint8_t	ZHighDetection		:1;
	uint8_t	ZLowDetection		:1;
	uint8_t	YHighDetection		:1;
	uint8_t	YLowDetection		:1;
	uint8_t	XHighDetection		:1;
	uint8_t	XLowDetection		:1;
} LSM9DS0_AccInterrupt_TypeDef;

typedef struct {
	I2C_TypeDef* I2C;
	uint8_t Address;
	uint8_t	InterruptPP_OD		:1;
	uint8_t	InterruptPolarity	:1;
	uint8_t InterruptLatching	:1;
	uint8_t MagIntEnable		:1;
	uint8_t	MagIntEnableX		:1;
	uint8_t	MagIntEnableY		:1;
	uint8_t	MagIntEnableZ		:1;
	uint8_t	Acc4DDetection		:1;
	uint8_t AccFIFOEnable		:1;
	uint8_t AccFIFOWatermarkEn	:1;
	uint8_t AccClickHPF			:1;
	uint8_t	AccHPFMode;
	uint8_t	AccHPFEnabled		:1;
	uint8_t AccInt1HPF			:1;
	uint8_t AccInt2HPF			:1;
	uint8_t	AccInt1Latching		:1;
	uint8_t	AccInt2Latching		:1;
	uint8_t AccDataRate;
	uint8_t AccBlockUpdate		:1;
	uint8_t AccAxisZEnable		:1;
	uint8_t AccAxisYEnable		:1;
	uint8_t AccAxisXEnable		:1;
	uint8_t AccBandwidth;
	uint8_t	AccFullScale;
	uint8_t	Int1onBoot			:1;
	uint8_t	Int1onTap			:1;
	uint8_t	Int1onAccInt1		:1;
	uint8_t	Int1onAccInt2		:1;
	uint8_t	Int1onMagInt		:1;
	uint8_t	Int1onAccDRDY		:1;
	uint8_t	Int1onMagDRDY		:1;
	uint8_t	Int1onFIFOEmpty		:1;

	uint8_t	Int2onTap			:1;
	uint8_t	Int2onAccInt1		:1;
	uint8_t	Int2onAccInt2		:1;
	uint8_t	Int2onMagInt		:1;
	uint8_t	Int2onAccDRDY		:1;
	uint8_t	Int2onMagDRDY		:1;
	uint8_t	Int2onFIFOEmpty		:1;
	uint8_t	Int2onFIFOWatermark	:1;

	uint8_t	TempSensorEnable	:1;

	uint8_t	MagResolution		:1;
	uint8_t	MagDataRate;
	uint8_t	MagFullScale;


	uint8_t	MagLowPowerMode		:1;
	uint8_t	MagSensorMode;

	uint8_t	AccFIFOMode;
	uint8_t AccFIFOWatermark	:5;

	LSM9DS0_AccInterrupt_TypeDef AccInt1;
	LSM9DS0_AccInterrupt_TypeDef AccInt2;

	uint8_t AccClickZDouble		:1;
	uint8_t	AccClickZSingle		:1;
	uint8_t AccClickYDouble		:1;
	uint8_t	AccClickYSingle		:1;
	uint8_t AccClickXDouble		:1;
	uint8_t	AccClickXSingle		:1;

} LSM9DS0_AccM_TypeDef;

int8_t LSM9DS0_GyroInit(LSM9DS0_Gyro_TypeDef* gyro, I2C_TypeDef* I2Cx, uint8_t addr, uint8_t scale, uint8_t bandwidth, uint8_t odr);
int8_t LSM9DS0_AccMInit(LSM9DS0_AccM_TypeDef* accM, I2C_TypeDef* I2Cx, uint8_t addr, uint8_t acc_scale, uint8_t acc_odr,uint8_t acc_bw, uint8_t mag_scale, uint8_t mag_odr, uint8_t mag_res);

int8_t LSM9DS0_GyroConfigure(LSM9DS0_Gyro_TypeDef* gyro);
int8_t LSM9DS0_AccMConfigure(LSM9DS0_AccM_TypeDef* accM);
int16_t LSM9DS0_ReadTemp(LSM9DS0_AccM_TypeDef* accM);

int8_t LSM9DS0_GyroReadRaw(LSM9DS0_Gyro_TypeDef* gyro, int16_3D_t *result);
int8_t LSM9DS0_AccReadRaw(LSM9DS0_AccM_TypeDef *acc, int16_3D_t *result);
int8_t LSM9DS0_MagReadRaw(LSM9DS0_AccM_TypeDef *mag, int16_3D_t *result);

void LSM9DS0_GyroScaleData(LSM9DS0_Gyro_TypeDef* gyro, int16_3D_t *source, float_3D_t *result);
void LSM9DS0_AccScaleData(LSM9DS0_AccM_TypeDef* acc, int16_3D_t *source, float_3D_t *result);
void LSM9DS0_MagScaleData(LSM9DS0_AccM_TypeDef* mag, int16_3D_t *source, float_3D_t *result);


int8_t LSM9DS0_GyroRead(LSM9DS0_Gyro_TypeDef *gyro, float_3D_t *result);
int8_t LSM9DS0_AccRead(LSM9DS0_AccM_TypeDef *acc, float_3D_t *result);
int8_t LSM9DS0_MagRead(LSM9DS0_AccM_TypeDef *mag, float_3D_t *result);

#endif /* LSM9DS0_H_ */
