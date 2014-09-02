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
	uint8_t	DataRate;
	uint8_t Bandwidth;
	uint8_t Enabled		:1;
	uint8_t
} LSM9DS0_Gyro_TypeDef;

void LSM9DS0_GyroInit(I2C_TypeDef* I2Cx, uint8_t scale, uint8_t bandwidth, uint8_t odr);
void LSM9DS0_AccMInit(I2C_TypeDef* I2Cx, uint8_t acc_scale, uint8_t acc_odr,uint8_t acc_bw, uint8_t mag_scale, uint8_t mag_odr);

int16_t LSM9DS0_ReadTemp(I2C_TypeDef* I2Cx);
uint8_t LSM9DS0_GyroRead(I2C_TypeDef* I2Cx, float_3D_t *rotation);
uint8_t LSM9DS0_AccRead(I2C_TypeDef* I2Cx, float_3D_t *acceleration);
uint8_t LSM9DS0_MagRead(I2C_TypeDef* I2Cx, float_3D_t *);

#endif /* LSM9DS0_H_ */
