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
	int m = log10(num);
	int digit;
	float tolerance = .0001f;
	if (num<0)
	{
		*(fstr++)='-';
		num = - num;
	}
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
/*

void Task_GetAcceleration(void)
{
	char tekst[50] = "";
	char X[10] ="";
	char Y[10] ="";
	char Z[10] ="";
	static uint32_t count;
//	int X,Xf,Y,Yf,Z,Zf;
	float_3D_t acc = {0.0,0.0,0.0};
	float_3D_t mag = {0.0,0.0,0.0};
	float_3D_t gyr = {0.0,0.0,0.0};
//	msleep(500);
	count++;
	LSM9DS0_AccRead(&accM,&acc);
	LSM9DS0_MagRead(&accM,&mag);
	LSM9DS0_GyroRead(&gyro,&gyr);

	sprintf(tekst,"%lu\t",count);
	TM_USART_Puts(USART1,tekst);

	ftoa(acc.X,X);
	ftoa(acc.Y,Y);
	ftoa(acc.Z,Z);
//	X  = (int)( acc.X * 1000);
//	Y  = (int)( acc.Y * 1000);
//	Z  = (int)( acc.Z * 1000);
//	Xf = (int)(((acc.X>=0)?acc.X - X:-acc.X+X)*10000);
//	Yf = (int)(((acc.Y>=0)?acc.Y - Y:-acc.Y+Y)*10000);
//	Zf = (int)(((acc.Z>=0)?acc.Z - Z:-acc.Z+Z)*10000);
//	sprintf(tekst,"ACC: X: %d.%d\tY: %d.%d\tZ: %d.%d\t",X,Xf,Y,Yf,Z,Zf);
	sprintf(tekst,"ACC: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);

	ftoa(mag.X,X);
	ftoa(mag.Y,Y);
	ftoa(mag.Z,Z);
//	X  = (int) mag.X*1000;
//	Y  = (int) mag.Y*1000;
//	Z  = (int) mag.Z*1000;
//	Xf = (int)(((mag.X>=0)?mag.X - X:-mag.X+X)*10000);
//	Yf = (int)(((mag.Y>=0)?mag.Y - Y:-mag.Y+Y)*10000);
//	Zf = (int)(((mag.Z>=0)?mag.Z - Z:-mag.Z+Z)*10000);
//	sprintf(tekst,"MAG: X: %d.%d\tY: %d.%d\tZ: %d.%d\t" ,X,Xf,Y,Yf,Z,Zf);
	sprintf(tekst,"MAG: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);

	ftoa(gyr.X,X);
	ftoa(gyr.Y,Y);
	ftoa(gyr.Z,Z);
//	X  = (int) gyr.X*1000;
//	Y  = (int) gyr.Y*1000;
//	Z  = (int) gyr.Z*1000;
//	Xf = (int)(((gyr.X>=0)?gyr.X - X:-gyr.X+X)*10000);
//	Yf = (int)(((gyr.Y>=0)?gyr.Y - Y:-gyr.Y+Y)*10000);
//	Zf = (int)(((gyr.Z>=0)?gyr.Z - Z:-gyr.Z+Z)*10000);
//	sprintf(tekst,"GYR: X: %d.%d\tY: %d.%d\tZ: %d.%d\t",X,Xf,Y,Yf,Z,Zf);
	sprintf(tekst,"GYR: X: %s\tY: %s\tZ: %s\t",X,Y,Z);
	TM_USART_Puts(USART1,tekst);
	TM_USART_Puts(USART1,"\r\n");
}
*/

void Task_GetAcceleration(void)
{
	//int16_3D_t data = {0,0,0};
	float_3D_t data = {0.0,0.0,0.0};
	char temp[30];
	char X[10], Y[10], Z[10];
	LSM9DS0_AccRead(accM,&data);
	ftoa(data.X,X);
	ftoa(data.Y,Y);
	ftoa(data.Z,Z);
	sprintf(temp,"ACC:\tX:%s\tY:%s\tZ:%s\r\n",X,Y,Z);
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

	LSM9DS0_AccMConfigure(accM);

	LSM9DS0_GyroConfigure(gyro);

	STM_InitTasks();
	STM_AddTask(1000,&Task_UpdateDisplay);
//	ShowNumber(DISP_TOP,12345);
	STM_AddTask(100000,&Task_ReadTemperature);
	STM_AddTask(10000,&Task_GetAcceleration);
//	STM_AddTask(10000,&Task_GetRotation);
//	STM_AddTask(100000,&Task_GetMagnets);
	STM_AddTask(1000000,&Task_BlinkLED);
	while (1)
	{
		STM_ExecuteTasks();
	}
	return 0;
}
