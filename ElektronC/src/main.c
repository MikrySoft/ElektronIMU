#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_it.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_usart.h"
#include "hardware_init.h"
#include "main.h"
#include "LSM9DS0.h"
#include "displays.h"
#include "TaskHandler.h"


LSM9DS0_AccM_TypeDef _accM;
LSM9DS0_Gyro_TypeDef _gyro;

LSM9DS0_AccM_TypeDef* accM = &_accM;
LSM9DS0_Gyro_TypeDef* gyro = &_gyro;

char *ftoa(float num, char* fstr)
{
	if (num<0)
	{
		*(fstr++)='-';
		num = - num;
	}
	int m = log10(num);
	int digit;
	float tolerance = .0001f;
	while (num > 0 + tolerance)
	{
		float weight = pow(10.0f, m);
		digit = floor(num / weight);
		num -= (digit*weight);
		*(fstr++)= '0' + digit;
		if (m == 0)
        *(fstr++) = '.';
		m--;
	}
	*(fstr) = '\0';
	return fstr;
}

void Task_UpdateDisplay(void)
{
	UpdateDisplay(I2C2);
}

void Task_ReadTemperature(void)
{
	int16_t temp = LSM9DS0_ReadTemp(accM);
	ShowNumber(DISP_TOP,temp);
}

void Task_BlinkLED(void)
{
	GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
	GPIO_ToggleBits(GPIOA,GPIO_Pin_4);
}



void Task_GetAcceleration(void)
{
	char tekst[50], X[10], Y[10], Z[10];
	static uint32_t count = 0;
	float_3D_t data = {0.0,0.0,0.0};


	sprintf(tekst,"%lu\t",count++);
	TM_USART_Puts(USART1,tekst);

	LSM9DS0_AccRead(accM,&data);
	ftoa(data.X,X);
	ftoa(data.Y,Y);
	ftoa(data.Z,Z);
	sprintf(tekst,"ACC: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);

	LSM9DS0_MagRead(accM,&data);
	ftoa(data.X,X);
	ftoa(data.Y,Y);
	ftoa(data.Z,Z);
	sprintf(tekst,"MAG: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);

	LSM9DS0_GyroRead(gyro,&data);
	ftoa(data.X,X);
	ftoa(data.Y,Y);
	ftoa(data.Z,Z);
	sprintf(tekst,"GYR: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);

	TM_USART_Puts(USART1,"\r\n");
}

/*

void Task_GetAcceleration(void)
{
	static uint32_t count = 0;
//	int16_3D_t data = {0,0,0};
	float_3D_t fdata = {0.0,0.0,0.0};
	char temp[40];
	char X[10], Y[10], Z[10];
	int8_t res;
//	LSM9DS0_AccReadRaw(accM,&data);
//	LSM9DS0_AccScaleData(accM,&data,&fdata);
	res = LSM9DS0_AccRead(accM,&fdata);
	ftoa(fdata.X,X);
	ftoa(fdata.Y,Y);
	ftoa(fdata.Z,Z);
	sprintf(temp,"%6lu:\tACC:\tX:%s\tY:%s\tZ:%s\r\n",count++,X,Y,Z);
//	sprintf(temp,"ACC:\tX:%6d\t%s\tY:%6d\t%s\tZ:%d\t%s\r\n",data.X,X,data.Y,Y,data.Z,Z);
//	sprintf(temp,"%6lu:\tACC:\tX:%6d\tY:%6d\tZ:%6d\r\n",count++,data.X,data.Y,data.Z);
	TM_USART_Puts(USART1,temp);
}

void Task_GetMagnets(void)
{
	int16_3D_t data = {0.0,0.0,0.0};
	char temp[30];
	LSM9DS0_MagReadRaw(accM,&data);
	sprintf(temp,"MAG:\tX:%06d\tY:%06d\tZ:%06d\r\n",data.X,data.Y,data.Z);
	TM_USART_Puts(USART1,temp);
}

void Task_GetRotation(void)
{
	int16_3D_t data = {0.0,0.0,0.0};
	char temp[30];
	LSM9DS0_GyroReadRaw(gyro,&data);
	sprintf(temp,"GYRO:\tX:%06d\tY:%06d\tZ:%06d\r\n",data.X,data.Y,data.Z);
	TM_USART_Puts(USART1,temp);
}

*/
void Task_MemoryRead(void)
{

	static uint8_t addr = 0;
	uint8_t acc, gyro;
	uint8_t racc[14] = {0x0F, 0x12, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x2E, 0x30, 0x34, 0x38};
	uint8_t rgyr[14] = {0x0F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x2E, 0x30, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
	char temp[30];
	if (addr < 14)
	{
		gyro = TM_I2C_ReadReg(I2C1,GYRO_ADDR,rgyr[addr]);
		acc = TM_I2C_ReadReg(I2C1,ACCEL_ADDR,racc[addr]);
		sprintf(temp,"%d:\tACC: 0x%02X=0x%02X\tGYRO: 0x%02X=0x%02X\r\n",addr,racc[addr],acc,rgyr[addr],gyro);
		TM_USART_Puts(USART1,temp);
		ShowNumber(DISP_TOP |DISP_FILL| DISP_HEX | DISP_DIG_2,racc[addr]);
		ShowNumber(DISP_BOT |DISP_FILL| DISP_HEX | DISP_DIG_2,rgyr[addr]);
		ShowNumber(DISP_TOP | DISP_HEX | DISP_DIG_4, acc);
		ShowNumber(DISP_BOT | DISP_HEX | DISP_DIG_4, gyro);
		addr++;
	}


}

int main(void){

	LED_init();
	TM_I2C_Init(I2C2,TM_I2C_PinsPack_1,TM_I2C_CLOCK_STANDARD);
	TM_I2C_Init(I2C1,TM_I2C_PinsPack_1,TM_I2C_CLOCK_FAST_MODE);
	TM_USART_Init(USART1,TM_USART_PinsPack_1,115200);
	SysTick_Init();
	LSM9DS0_AccMInit(accM,I2C1,ACCEL_ADDR,
			LSM9DS0_ACC_FULL_SCALE_2G,
			LSM9DS0_ACC_AODR_25HZ,
			LSM9DS0_ACC_ANTI_ALIAS_BW_733HZ,
			LSM9DS0_MAG_FULL_SCALE_2G,
			LSM9DS0_MAG_DATA_RATE_50HZ,
			LSM9DS0_MAG_RESOLUTION_LOW);

	LSM9DS0_GyroInit(gyro,I2C1,GYRO_ADDR,
			LSM9DS0_GYRO_FS_245DPS,
			LSM9DS0_GYRO_BW_LOW,
			LSM9DS0_GYRO_ODR_95HZ);

//	LSM9DS0_AccMConfigure(accM);
//	LSM9DS0_GyroConfigure(gyro);
//	ShowNumber(DISP_TOP | DISP_HEX | DISP_DIG_2, TM_I2C_ReadReg(I2C1,ACCEL_ADDR,LSM9DS0_ACC_CTRL_REG2));
//	ShowNumber(DISP_TOP | DISP_HEX | DISP_DIG_4, TM_I2C_ReadReg(I2C1,ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG7));
//	ShowNumber(			  DISP_HEX | DISP_DIG_2, TM_I2C_ReadReg(I2C1,ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG5));
//	ShowNumber(			  DISP_HEX | DISP_DIG_4, TM_I2C_ReadReg(I2C1,ACCEL_ADDR,LSM9DS0_MAG_CTRL_REG6));

	STM_InitTasks();
	STM_AddTask(1000,&Task_UpdateDisplay);
	STM_AddTask(100000,&Task_MemoryRead);
//	STM_AddTask(100000,&Task_ReadTemperature);
//	STM_AddTask(100000,&Task_GetAcceleration);
	STM_AddTask(1000000,&Task_BlinkLED);
	while (1)
	{
		STM_ExecuteTasks();
	}
	return 0;
}
