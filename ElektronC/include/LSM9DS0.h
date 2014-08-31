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

void LSM9DS0_GyroInit(I2C_TypeDef* I2Cx, uint8_t scale, uint8_t bandwidth, uint8_t odr);
void LSM9DS0_AccMInit(I2C_TypeDef* I2Cx, uint8_t acc_scale, uint8_t acc_odr,uint8_t acc_bw, uint8_t mag_scale, uint8_t mag_odr);

int16_t LSM9DS0_ReadTemp(I2C_TypeDef* I2Cx);

#endif /* LSM9DS0_H_ */
