/*
 * LSM9DS0.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mikrysoft
 */


#include "LSM9DS0.h"
#include "stm32f4xx_it.h"
#include "tm_stm32f4_i2c.h"

int8_t LSM9DS0_GyroInit(LSM9DS0_Gyro_TypeDef* gyro, I2C_TypeDef* I2Cx, uint8_t addr, uint8_t scale, uint8_t bandwidth, uint8_t odr)
{
	if (!gyro) return -1;
	if (!I2Cx) return -2;

	gyro->Address   = addr;
	gyro->Bandwidth = bandwidth;
	gyro->DataRate  = odr;
	gyro->FullScale = scale;
	gyro->I2C 		= I2Cx;

	gyro->Enabled 				= 1;
	gyro->XAxisEnabled			= 1;
	gyro->YAxisEnabled			= 1;
	gyro->ZAxisEnabled			= 1;
	gyro->HighPassEnable		= 0;
	gyro->HighPassMode  		= LSM9DS0_GYRO_HPM_NORMAL_RESET;
	gyro->HighPassCutoff		= LSM9DS0_GYRO_HPCF_DIV1;
	gyro->InterruptEnabled 		= 0;
	gyro->InterruptOnBoot 		= 0;
	gyro->InterruptPolarity		= 0;
	gyro->InterruptPP_OD		= 0;
	gyro->DRDYOnDRDY			= 0;
	gyro->FIFOWatermarkOnDRDY	= 0;
	gyro->FIFOOverrunOnDRDY		= 0;
	gyro->FIFOEmptyOnDRDY		= 0;
	gyro->BlockUpdate			= 0;
	gyro->BigLittleEndian		= 0;
	gyro->FIFOEnable			= 0;
	gyro->FIFOMode				= LSM9DS0_GYRO_FIFO_CTRL_FM_BYPASS;
	gyro->FIFOWatermark			= 0;
	gyro->IntSource				= LSM9DS0_GYRO_INT1_SRC_LPF1;
	gyro->OutSource				= LSM9DS0_GYRO_OUT_SRC_LPF1;
	gyro->IntAndOrMode			= 0;
	gyro->IntLatching			= 0;
	gyro->IntZHighEnable		= 0;
	gyro->IntZLowEnable			= 0;
	gyro->IntYHighEnable		= 0;
	gyro->IntYLowEnable			= 0;
	gyro->IntXHighEnable		= 0;
	gyro->IntXLowEnable			= 0;
	LSM9DS0_GyroConfigure(gyro);
	return 0;
}

int8_t LSM9DS0_GyroConfigure(LSM9DS0_Gyro_TypeDef* gyro)
{
	uint8_t regdata;
	if (!gyro) return -1;
	if (!(gyro->I2C)) return -2;
	TM_I2C_WriteReg(gyro->I2C,gyro->Address,LSM9DS0_GYRO_CTRL_REG5,LSM9DS0_GYRO_CTRL_REG5_BOOT);
	usleep(500);
	TM_I2C_WriteReg(gyro->I2C,gyro->Address,LSM9DS0_GYRO_CTRL_REG5,0x00);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->DataRate & LSM9DS0_GYRO_CTRL_REG1_DR);
	regdata |= (gyro->Bandwidth & LSM9DS0_GYRO_CTRL_REG1_BW);
	regdata |= (gyro->Enabled)?LSM9DS0_GYRO_CTRL_REG1_PD:0;
	regdata |= (gyro->XAxisEnabled)?LSM9DS0_GYRO_CTRL_REG1_XEN:0;
	regdata |= (gyro->YAxisEnabled)?LSM9DS0_GYRO_CTRL_REG1_YEN:0;
	regdata |= (gyro->ZAxisEnabled)?LSM9DS0_GYRO_CTRL_REG1_ZEN:0;
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_CTRL_REG1, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->HighPassMode & LSM9DS0_GYRO_CTRL_REG2_HPM);
	regdata |= (gyro->HighPassCutoff &LSM9DS0_GYRO_CTRL_REG2_HPCF);
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_CTRL_REG2, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->InterruptEnabled)?LSM9DS0_GYRO_CTRL_REG3_I1_INT1:0;
	regdata |= (gyro->InterruptOnBoot)?LSM9DS0_GYRO_CTRL_REG3_I1_BOOT:0;
	regdata |= (gyro->InterruptPolarity)?LSM9DS0_GYRO_CTRL_REG3_H_Lactive:0;
	regdata |= (gyro->InterruptPP_OD)?LSM9DS0_GYRO_CTRL_REG3_PP_OD:0;
	regdata |= (gyro->DRDYOnDRDY)?LSM9DS0_GYRO_CTRL_REG3_I2_DRDY:0;
	regdata |= (gyro->FIFOWatermarkOnDRDY)?LSM9DS0_GYRO_CTRL_REG3_I2_WTM:0;
	regdata |= (gyro->FIFOOverrunOnDRDY)?LSM9DS0_GYRO_CTRL_REG3_I2_ORun:0;
	regdata |= (gyro->FIFOEmptyOnDRDY)?LSM9DS0_GYRO_CTRL_REG3_I2_Empty:0;
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_CTRL_REG3, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->BlockUpdate)?LSM9DS0_GYRO_CTRL_REG4_BDU:0;
	regdata |= (gyro->BigLittleEndian)?LSM9DS0_GYRO_CTRL_REG4_BLE:0;
	regdata |= (gyro->FullScale & LSM9DS0_GYRO_CTRL_REG4_FS);
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_CTRL_REG4, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->FIFOEnable)?LSM9DS0_GYRO_CTRL_REG5_FIFO_EN:0;
	regdata |= (gyro->HighPassEnable)?LSM9DS0_GYRO_CTRL_REG5_H_PEN:0;
	regdata |= (gyro->IntSource & LSM9DS0_GYRO_CTRL_REG5_INT1_Sel);
	regdata |= (gyro->OutSource & LSM9DS0_GYRO_CTRL_REG5_OUT_Sel);
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_CTRL_REG5, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->FIFOMode& LSM9DS0_GYRO_FIFO_CTRL_FM);
	regdata |= (gyro->FIFOWatermark& LSM9DS0_GYRO_FIFO_CTRL_WTM);
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_FIFO_CTRL, regdata);
	usleep(100);
	regdata = 0x00;
	regdata |= (gyro->IntAndOrMode)?LSM9DS0_GYRO_INT1_CFG_AOR:0;
	regdata |= (gyro->IntLatching)?LSM9DS0_GYRO_INT1_CFG_LIR:0;
	regdata |= (gyro->IntZHighEnable)?LSM9DS0_GYRO_INT1_CFG_ZHIE:0;
	regdata |= (gyro->IntZLowEnable)?LSM9DS0_GYRO_INT1_CFG_ZLIE:0;
	regdata |= (gyro->IntYHighEnable)?LSM9DS0_GYRO_INT1_CFG_YHIE:0;
	regdata |= (gyro->IntYLowEnable)?LSM9DS0_GYRO_INT1_CFG_YLIE:0;
	regdata |= (gyro->IntXHighEnable)?LSM9DS0_GYRO_INT1_CFG_XHIE:0;
	regdata |= (gyro->IntXLowEnable)?LSM9DS0_GYRO_INT1_CFG_XLIE:0;
	TM_I2C_WriteReg(gyro->I2C, gyro->Address, LSM9DS0_GYRO_INT1_CFG, regdata);
	usleep(100);
	return 0;
}

int8_t LSM9DS0_AccMInit(LSM9DS0_AccM_TypeDef* accM, I2C_TypeDef* I2Cx, uint8_t addr, uint8_t acc_scale, uint8_t acc_odr,uint8_t acc_bw, uint8_t mag_scale, uint8_t mag_odr, uint8_t mag_res)
{
	if (!accM) return -1;
	if (!I2Cx) return -2;
	accM->Address = addr;
	accM->I2C = I2Cx;
	accM->InterruptPP_OD 		=0;
	accM->InterruptPolarity 	=0;
	accM->InterruptLatching		=0;

	accM->MagIntEnable			=0;
	accM->MagIntEnableX			=0;
	accM->MagIntEnableY			=0;
	accM->MagIntEnableZ			=0;

	accM->Acc4DDetection		=0;
	accM->AccFIFOEnable			=0;
	accM->AccFIFOWatermarkEn	=0;

	accM->AccClickHPF			=0;
	accM->AccHPFMode			=0;
	accM->AccHPFEnabled			=0;

	accM->AccInt1HPF			=0;
	accM->AccInt2HPF			=0;
	accM->AccInt1Latching		=0;
	accM->AccInt2Latching		=0;

	accM->AccDataRate			=acc_odr;
	accM->AccBlockUpdate		=0;
	accM->AccAxisZEnable		=1;
	accM->AccAxisYEnable		=1;
	accM->AccAxisXEnable		=1;
	accM->AccBandwidth			=acc_bw;
	accM->AccFullScale			=acc_scale;

	accM->Int1onBoot			=0;
	accM->Int1onTap				=0;
	accM->Int1onAccInt1			=0;
	accM->Int1onAccInt2			=0;
	accM->Int1onMagInt			=0;
	accM->Int1onAccDRDY			=0;
	accM->Int1onMagDRDY			=0;
	accM->Int1onFIFOEmpty		=0;

	accM->Int2onTap				=0;
	accM->Int2onAccInt1			=0;
	accM->Int2onAccInt2			=0;
	accM->Int2onMagInt			=0;
	accM->Int2onAccDRDY			=0;
	accM->Int2onMagDRDY			=0;
	accM->Int2onFIFOEmpty		=0;
	accM->Int2onFIFOWatermark	=0;

	accM->TempSensorEnable		=1;

	accM->MagResolution			=mag_res;
	accM->MagDataRate			=mag_odr;
	accM->MagFullScale			=mag_scale;


	accM->MagLowPowerMode		=0;
	accM->MagSensorMode			=0;

	accM->AccFIFOMode			=0;
	accM->AccFIFOWatermark		=0;

	accM->AccInt1.AndOrMode			=0;
	accM->AccInt1.Detect6DMode		=0;
	accM->AccInt1.ZHighDetection	=0;
	accM->AccInt1.ZLowDetection		=0;
	accM->AccInt1.YHighDetection	=0;
	accM->AccInt1.YLowDetection		=0;
	accM->AccInt1.XHighDetection	=0;
	accM->AccInt1.XLowDetection		=0;

	accM->AccInt2.AndOrMode			=0;
	accM->AccInt2.Detect6DMode		=0;
	accM->AccInt2.ZHighDetection	=0;
	accM->AccInt2.ZLowDetection		=0;
	accM->AccInt2.YHighDetection	=0;
	accM->AccInt2.YLowDetection		=0;
	accM->AccInt2.XHighDetection	=0;
	accM->AccInt2.XLowDetection		=0;

	accM->AccClickZDouble				=0;
	accM->AccClickZSingle				=0;
	accM->AccClickYDouble				=0;
	accM->AccClickYSingle				=0;
	accM->AccClickXDouble				=0;
	accM->AccClickXSingle				=0;
	LSM9DS0_AccMConfigure(accM);
	return 0;

}

int8_t LSM9DS0_AccMConfigure(LSM9DS0_AccM_TypeDef* accM)
{
	uint8_t regdata;
	if (!accM) return -1;
	if (!(accM->I2C)) return -2;
	TM_I2C_WriteReg(accM->I2C, accM->Address, LSM9DS0_ACC_CTRL_REG0,LSM9DS0_ACC_CTRL_REG0_BOOT);
	usleep(500);
	TM_I2C_WriteReg(accM->I2C, accM->Address, LSM9DS0_ACC_CTRL_REG0,0x00);
	regdata = 0;
	regdata |= (accM->InterruptPP_OD)? 		LSM9DS0_MAG_INT_CFG_PP_OD	:0;
	regdata |= (accM->InterruptPolarity)?	LSM9DS0_MAG_INT_CFG_IEA		:0;
	regdata |= (accM->InterruptLatching)?	LSM9DS0_MAG_INT_CFG_IEL		:0;
	regdata |= (accM->MagIntEnable)? 		LSM9DS0_MAG_INT_CFG_MIEN	:0;
	regdata |= (accM->MagIntEnableX)?		LSM9DS0_MAG_INT_CFG_XMIEN	:0;
	regdata |= (accM->MagIntEnableY)?		LSM9DS0_MAG_INT_CFG_YMIEN	:0;
	regdata |= (accM->MagIntEnableZ)?		LSM9DS0_MAG_INT_CFG_ZMIEN	:0;
	regdata |= (accM->Acc4DDetection)?		LSM9DS0_MAG_INT_CFG_4D		:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_INT_CFG, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccFIFOEnable)?		LSM9DS0_ACC_CTRL_REG0_FIFO_EN	:0;
	regdata |= (accM->AccFIFOWatermarkEn)?	LSM9DS0_ACC_CTRL_REG0_WTM_EN	:0;
	regdata |= (accM->AccClickHPF)?			LSM9DS0_ACC_CTRL_REG0_HP_Click	:0;
	regdata |= (accM->AccInt1HPF)?			LSM9DS0_ACC_CTRL_REG0_HPIS1		:0;
	regdata |= (accM->AccInt2HPF)?			LSM9DS0_ACC_CTRL_REG0_HIPS2		:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_CTRL_REG0, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccDataRate & 		LSM9DS0_ACC_CTRL_REG1_AODR);
	regdata |= (accM->AccBlockUpdate)?		LSM9DS0_ACC_CTRL_REG1_BDU		:0;
	regdata |= (accM->AccAxisZEnable)?		LSM9DS0_ACC_CTRL_REG1_AZEN		:0;
	regdata |= (accM->AccAxisYEnable)?		LSM9DS0_ACC_CTRL_REG1_AYEN		:0;
	regdata |= (accM->AccAxisXEnable)?		LSM9DS0_ACC_CTRL_REG1_AXEN		:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_CTRL_REG1, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccBandwidth &		LSM9DS0_ACC_CTRL_REG2_ABW);
	regdata |= (accM->AccFullScale & 		LSM9DS0_ACC_CTRL_REG2_AFS);
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_CTRL_REG2, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->Int1onBoot)?			LSM9DS0_MAG_CTRL_REG3_P1_BOOT	:0;
	regdata |= (accM->Int1onTap)?			LSM9DS0_MAG_CTRL_REG3_P1_TAP	:0;
	regdata |= (accM->Int1onAccInt1)?		LSM9DS0_MAG_CTRL_REG3_P1_INT1	:0;
	regdata |= (accM->Int1onAccInt2)?		LSM9DS0_MAG_CTRL_REG3_P1_INT2	:0;
	regdata |= (accM->Int1onMagInt)?		LSM9DS0_MAG_CTRL_REG3_P1_INTM	:0;
	regdata |= (accM->Int1onAccDRDY)?		LSM9DS0_MAG_CTRL_REG3_P1_DRDYA	:0;
	regdata |= (accM->Int1onMagDRDY)?		LSM9DS0_MAG_CTRL_REG3_P1_DRDYM	:0;
	regdata |= (accM->Int1onFIFOEmpty)?		LSM9DS0_MAG_CTRL_REG3_P1_EMPTY	:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_CTRL_REG3, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->Int2onTap)?			LSM9DS0_MAG_CTRL_REG4_P2_TAP	:0;
	regdata |= (accM->Int2onAccInt1)?		LSM9DS0_MAG_CTRL_REG4_P2_INT1	:0;
	regdata |= (accM->Int2onAccInt2)?		LSM9DS0_MAG_CTRL_REG4_P2_INT2	:0;
	regdata |= (accM->Int2onMagInt)?		LSM9DS0_MAG_CTRL_REG4_P2_INTM	:0;
	regdata |= (accM->Int2onAccDRDY)?		LSM9DS0_MAG_CTRL_REG4_P2_DRDYA	:0;
	regdata |= (accM->Int2onMagDRDY)?		LSM9DS0_MAG_CTRL_REG4_P2_DRDYM	:0;
	regdata |= (accM->Int2onFIFOEmpty)?		LSM9DS0_MAG_CTRL_REG4_P2_EMPTY	:0;
	regdata |= (accM->Int2onFIFOWatermark)?	LSM9DS0_MAG_CTRL_REG4_P2_WTM	:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_CTRL_REG4, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->TempSensorEnable)?	LSM9DS0_MAG_CTRL_REG5_TEMP_EN	:0;
	regdata |= (accM->MagResolution)?		LSM9DS0_MAG_CTRL_REG5_M_RES		:0;
	regdata |= (accM->MagDataRate & 		LSM9DS0_MAG_CTRL_REG5_M_ODR);
	regdata |= (accM->AccInt1Latching)?		LSM9DS0_MAG_CTRL_REG5_LIR2		:0;
	regdata |= (accM->AccInt2Latching)?		LSM9DS0_MAG_CTRL_REG5_LIR1		:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_CTRL_REG5, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->MagFullScale & 		LSM9DS0_MAG_CTRL_REG6_MFS);
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_CTRL_REG6, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccHPFMode & 			LSM9DS0_MAG_CTRL_REG7_AHPM);
	regdata |= (accM->AccHPFEnabled)?		LSM9DS0_MAG_CTRL_REG7_AFDS		:0;
	regdata |= (accM->MagLowPowerMode)?		LSM9DS0_MAG_CTRL_REG7_MLP		:0;
	regdata |= (accM->MagSensorMode &		LSM9DS0_MAG_CTRL_REG7_MD);
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_MAG_CTRL_REG7, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccFIFOMode &			LSM9DS0_ACC_FIFO_CFG_FM);
	regdata |= (accM->AccFIFOWatermark &	LSM9DS0_ACC_FIFO_CFG_FTH );
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_FIFO_CFG, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccInt1.AndOrMode)?		LSM9DS0_ACC_INT1_GEN_AOI:0;
	regdata |= (accM->AccInt1.Detect6DMode)?	LSM9DS0_ACC_INT1_GEN_6D:0;
	regdata |= (accM->AccInt1.ZHighDetection)?	LSM9DS0_ACC_INT1_GEN_ZHIE_ZUPE:0;
	regdata |= (accM->AccInt1.ZLowDetection)?	LSM9DS0_ACC_INT1_GEN_ZLIE_ZDOWNE:0;
	regdata |= (accM->AccInt1.YHighDetection)?	LSM9DS0_ACC_INT1_GEN_YHIE_YUPE:0;
	regdata |= (accM->AccInt1.YLowDetection)?	LSM9DS0_ACC_INT1_GEN_YLIE_YDOWNE:0;
	regdata |= (accM->AccInt1.XHighDetection)?	LSM9DS0_ACC_INT1_GEN_XHIE_XUPE:0;
	regdata |= (accM->AccInt1.XLowDetection)?	LSM9DS0_ACC_INT1_GEN_XLIE_XDOWNE:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_INT1_GEN, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccInt2.AndOrMode)?		LSM9DS0_ACC_INT2_GEN_AOI:0;
	regdata |= (accM->AccInt2.Detect6DMode)?	LSM9DS0_ACC_INT2_GEN_6D:0;
	regdata |= (accM->AccInt2.ZHighDetection)?	LSM9DS0_ACC_INT2_GEN_ZHIE_ZUPE:0;
	regdata |= (accM->AccInt2.ZLowDetection)?	LSM9DS0_ACC_INT2_GEN_ZLIE_ZDOWNE:0;
	regdata |= (accM->AccInt2.YHighDetection)?	LSM9DS0_ACC_INT2_GEN_YHIE_YUPE:0;
	regdata |= (accM->AccInt2.YLowDetection)?	LSM9DS0_ACC_INT2_GEN_YLIE_YDOWNE:0;
	regdata |= (accM->AccInt2.XHighDetection)?	LSM9DS0_ACC_INT2_GEN_XHIE_XUPE:0;
	regdata |= (accM->AccInt2.XLowDetection)?	LSM9DS0_ACC_INT2_GEN_XLIE_XDOWNE:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_INT2_GEN, regdata);
	usleep(100);
	regdata = 0;
	regdata |= (accM->AccClickZDouble)?	LSM9DS0_ACC_CLICK_CFG_ZD:0;
	regdata |= (accM->AccClickZSingle)?	LSM9DS0_ACC_CLICK_CFG_ZS:0;
	regdata |= (accM->AccClickYDouble)?	LSM9DS0_ACC_CLICK_CFG_YD:0;
	regdata |= (accM->AccClickYSingle)?	LSM9DS0_ACC_CLICK_CFG_YS:0;
	regdata |= (accM->AccClickXDouble)?	LSM9DS0_ACC_CLICK_CFG_XD:0;
	regdata |= (accM->AccClickXSingle)?	LSM9DS0_ACC_CLICK_CFG_XS:0;
	TM_I2C_WriteReg(accM->I2C,accM->Address,LSM9DS0_ACC_CLICK_SRC, regdata);
	usleep(100);
	return 0;
}


int16_t LSM9DS0_ReadTemp(LSM9DS0_AccM_TypeDef* accM)
{
	int16_t r;
	uint8_t data[LSM9DS0_TEMP_COUNT];
	TM_I2C_ReadMulti(accM->I2C,accM->Address,LSM9DS0_TEMP_ADDR,data,LSM9DS0_TEMP_COUNT);
	r = ((data[1]<<8)|data[0])&0x0FFF;
	r |= (r>>11)?0xF000:0x0000;
	return r;
}


int8_t LSM9DS0_GyroRead(LSM9DS0_Gyro_TypeDef* gyro, float_3D_t* rotation)
{
	float mult=0;
	uint8_t data[LSM9DS0_GYRO_COUNT];
	int16_t x,y,z;
	if (!gyro) return -1;
	if (!(gyro->I2C)) return -2;
	if (!rotation) return -3;
	switch (gyro->FullScale){
		case LSM9DS0_GYRO_FS_245DPS: mult = LSM9DS0_GYRO_FS_MULTIPLIER_245DPS; break;
		case LSM9DS0_GYRO_FS_500DPS: mult = LSM9DS0_GYRO_FS_MULTIPLIER_500DPS; break;
		case LSM9DS0_GYRO_FS_2000DPS: mult = LSM9DS0_GYRO_FS_MULTIPLIER_2000DPS; break;
	}
	TM_I2C_ReadMulti(gyro->I2C,gyro->Address,LSM9DS0_GYRO_ADDR,data,LSM9DS0_GYRO_COUNT);
	x = (data[1]<<8)|data[0];
	y = (data[3]<<8)|data[2];
	z = (data[5]<<8)|data[4];
	rotation->X = x*mult;
	rotation->Y = y*mult;
	rotation->Z = z*mult;
	return 0;
}

int8_t LSM9DS0_AccRead(LSM9DS0_AccM_TypeDef* accel, float_3D_t* acceleration)
{
	float mult=0;
	uint8_t data[LSM9DS0_ACC_OUT_COUNT];
	int16_t x,y,z;
	if (!accel) return -1;
	if (!(accel->I2C)) return -2;
	if (!acceleration) return -3;
	switch (accel->AccFullScale){
		case LSM9DS0_ACC_FULL_SCALE_2G: mult = LSM9DS0_ACC_MULTIPLIER_2G; break;
		case LSM9DS0_ACC_FULL_SCALE_4G: mult = LSM9DS0_ACC_MULTIPLIER_4G; break;
		case LSM9DS0_ACC_FULL_SCALE_6G: mult = LSM9DS0_ACC_MULTIPLIER_6G; break;
		case LSM9DS0_ACC_FULL_SCALE_8G: mult = LSM9DS0_ACC_MULTIPLIER_8G; break;
		case LSM9DS0_ACC_FULL_SCALE_16G: mult = LSM9DS0_ACC_MULTIPLIER_16G; break;
	}
	TM_I2C_ReadMulti(accel->I2C,accel->Address,LSM9DS0_ACC_OUT_ADDR,data,LSM9DS0_ACC_OUT_COUNT);

	x = (data[1]<<8)|data[0];
	y = (data[3]<<8)|data[2];
	z = (data[5]<<8)|data[4];
	acceleration->X = x*mult;
	acceleration->Y = y*mult;
	acceleration->Z = z*mult;
	return 0;

}

int8_t LSM9DS0_MagRead(LSM9DS0_AccM_TypeDef* mag, float_3D_t* magnetism)
{

	float mult = 0;
	uint8_t data[LSM9DS0_MAG_COUNT];
	int16_t x,y,z;
	if (!mag) return -1;
	if (!(mag->I2C)) return -2;
	if (!magnetism) return -3;
	switch (mag->MagFullScale){
		case LSM9DS0_MAG_FULL_SCALE_2G: mult = LSM9DS0_MAG_MULTIPLIER_2G; break;
		case LSM9DS0_MAG_FULL_SCALE_4G: mult = LSM9DS0_MAG_MULTIPLIER_4G; break;
		case LSM9DS0_MAG_FULL_SCALE_8G: mult = LSM9DS0_MAG_MULTIPLIER_8G; break;
		case LSM9DS0_MAG_FULL_SCALE_12G: mult = LSM9DS0_MAG_MULTIPLIER_12G; break;
	}
	TM_I2C_ReadMulti(mag->I2C,mag->Address,LSM9DS0_MAG_ADDR,data,LSM9DS0_MAG_COUNT);
	x = (data[1]<<8)|data[0];
	y = (data[3]<<8)|data[2];
	z = (data[5]<<8)|data[4];
	magnetism->X = x*mult;
	magnetism->Y = y*mult;
	magnetism->Z = z*mult;
	return 0;

}

