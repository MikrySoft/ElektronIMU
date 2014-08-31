/*
 * LSM9DS0.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */


#include "LSM9DS0.h"
#include "tm_stm32f4_i2c.h"

void LSM9DS0_GyroInit(I2C_TypeDef* I2Cx, uint8_t scale, uint8_t bandwidth, uint8_t odr)
{
	TM_I2C_WriteReg	(I2Cx, GYRO_ADDR,LSM9DS0_GYRO_CTRL_REG1,
						(odr & LSM9DS0_GYRO_CTRL_REG1_DR) |
						(bandwidth & LSM9DS0_GYRO_CTRL_REG1_BW)|
						LSM9DS0_GYRO_CTRL_REG1_PD|
						LSM9DS0_GYRO_CTRL_REG1_ZEN|
						LSM9DS0_GYRO_CTRL_REG1_YEN|
						LSM9DS0_GYRO_CTRL_REG1_XEN
					);
	TM_I2C_WriteReg	(I2Cx, GYRO_ADDR,LSM9DS0_GYRO_CTRL_REG4,
						(scale & LSM9DS0_GYRO_CTRL_REG4_FS)
					);

}
void LSM9DS0_AccMInit(I2C_TypeDef* I2Cx, uint8_t acc_scale, uint8_t acc_odr,uint8_t acc_bw, uint8_t mag_scale, uint8_t mag_odr)
{
	TM_I2C_WriteReg	(I2Cx, ACCEL_ADDR,LSM9DS0_ACC_CTRL_REG1,
						(acc_odr&LSM9DS0_ACC_CTRL_REG1_AODR)|
						LSM9DS0_ACC_CTRL_REG1_AZEN|
						LSM9DS0_ACC_CTRL_REG1_AYEN|
						LSM9DS0_ACC_CTRL_REG1_AXEN
					);
	TM_I2C_WriteReg	(I2Cx, ACCEL_ADDR,LSM9DS0_ACC_CTRL_REG2,
						(acc_bw & LSM9DS0_ACC_CTRL_REG2_ABW)|
						(acc_scale & LSM9DS0_ACC_CTRL_REG2_AFS)
					);

	TM_I2C_WriteReg	(I2Cx, ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG5,
						LSM9DS0_MAG_CTRL_REG5_TEMP_EN |
						lSM9DS0_MAG_RESOLUTION_HIGH |
						(mag_odr & LSM9DS0_MAG_CTRL_REG5_M_ODR)
					);

	TM_I2C_WriteReg	(I2Cx, ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG6,
						(mag_scale&LSM9DS0_MAG_CTRL_REG6_MFS)
					);
	TM_I2C_WriteReg	(I2Cx, ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG7, LSM9DS0_MAG_MODE_CONTINUOUS
					);

}


int16_t LSM9DS0_ReadTemp(I2C_TypeDef* I2Cx)
{
	uint8_t h,l;
	int16_t r;
	h = TM_I2C_ReadReg(I2Cx,ACCEL_ADDR,LSM9DS0_TEMP_H);
	l = TM_I2C_ReadReg(I2Cx,ACCEL_ADDR,LSM9DS0_TEMP_L);
	r = ((h<<8)|l)&0x0FFF;
	r |= (r>>11)?0xF000:0x0000;
	return r;
}
