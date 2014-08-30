#include "tm_stm32f4_dac.h"

void TM_DAC_Init(TM_DAC_Channel_t DACx) {
	GPIO_InitTypeDef GPIO_InitDef;
	DAC_InitTypeDef DAC_InitDef;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	if (DACx == TM_DAC1) {
		GPIO_InitDef.GPIO_Pin = GPIO_Pin_4;
	} else {
		GPIO_InitDef.GPIO_Pin = GPIO_Pin_5;
	}
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitDef);
	
	
	DAC_InitDef.DAC_Trigger = DAC_Trigger_None;
	DAC_InitDef.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitDef.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	if (DACx == TM_DAC1) {
		DAC_Init(DAC_Channel_1, &DAC_InitDef);
		DAC_Cmd(DAC_Channel_1, ENABLE);
	} else {
		DAC_Init(DAC_Channel_2, &DAC_InitDef);
		DAC_Cmd(DAC_Channel_2, ENABLE);
	}
}

void TM_DAC_SetValue(TM_DAC_Channel_t DACx, uint16_t value) {
	if (value > 4095) {
		value = 4095;
	}
	if (DACx == TM_DAC1) {
		DAC_SetChannel1Data(DAC_Align_12b_R, value);
	} else {
		DAC_SetChannel2Data(DAC_Align_12b_R, value);
	}
		
}

