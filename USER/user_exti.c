#include "user_exti.h"
#include <stdio.h>
#include <stdlib.h>

// typedef void (*exti_handler_cb_t)(void);
exti_handler_cb_t exti_line15_cb = NULL;
exti_handler_cb_t exti_line8_cb = NULL;			// from nRF24
//exti_handler_cb exti_line10_cb = NULL;

extern NVIC_InitTypeDef NVIC_InitStructure;
extern EXTI_InitTypeDef EXTI_InitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;
	
void EXTI_Setup(void){
	// EXTI Clock Settings
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
	// MPU6050 / HMC5883L (I2C)
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// NRF24L01 (SPI)
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource15);
	EXTI_DeInit();
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);
}

void EXTI15_10_IRQHandler(void){
	if (EXTI_GetITStatus(EXTI_Line15) != RESET){
		if (exti_line15_cb != NULL)
			exti_line15_cb();
        EXTI_ClearITPendingBit(EXTI_Line15);
	}
}

void EXTI9_5_IRQHandler(void){
	if (EXTI_GetITStatus(EXTI_Line8) != RESET){
		if (exti_line8_cb != NULL)
			exti_line8_cb();
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

void EXTI_IRQnHanlder_RegisterCB(uint32_t EXTI_Line, exti_handler_cb_t cb){
	if (EXTI_Line == EXTI_Line15){
		exti_line15_cb = cb;
	} else if (EXTI_Line == EXTI_Line8){
		exti_line8_cb = cb;
	}
}

void EXTI_IRQnHanlder_DeRegisterCB(uint32_t EXTI_Line){
	if (EXTI_Line == EXTI_Line15){
		exti_line15_cb = NULL;
	} else if (EXTI_Line == EXTI_Line8){
		exti_line8_cb = NULL;
	}
}
