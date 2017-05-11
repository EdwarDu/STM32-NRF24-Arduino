/** [TODO] Use TIM5 on the board with
 * TIM5_CH1 @ PA0   USART2_CTS
 * TIM5_CH2 @ PA1   USART2_RTS
 * TIM5_CH3 @ PA2   USART2_TX
 * TIM5_CH4 @ PA3   USART2_RX     GPS module needs to be relocated to UART4 or 5 */

#include "user_pwm.h"
#include <stdio.h>
#include <stdlib.h>

extern NVIC_InitTypeDef 			NVIC_InitStructure;
extern TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef			TIM_OCInitStructure;
extern GPIO_InitTypeDef			GPIO_InitStructure;

#define MIN_THROTTLE		199
#define MAX_THROTTLE		399
#define THROTTLE_RANGE		200

void TIM5_PWM_Setup(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_UP_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/
	
	TIM_DeInit(TIM5);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 360;  // 72MHz / 720 = 100KHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 3999; //I need 50Hz PWM signal
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = /*MAX_THROTTLE; //For Throttle Calibration for now*/ MIN_THROTTLE; // 1ms Lowest Throttle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

// th [399, 799]
void TIM5_PWM_Set_Throttle(uint8_t ch, uint8_t th){
	switch(ch){
		case 1: TIM_SetCompare1(TIM5, MIN_THROTTLE + th * THROTTLE_RANGE / 256); break;
		case 2: TIM_SetCompare2(TIM5, MIN_THROTTLE + th * THROTTLE_RANGE / 256); break;
		case 3: TIM_SetCompare3(TIM5, MIN_THROTTLE + th * THROTTLE_RANGE / 256); break;
		case 4: TIM_SetCompare4(TIM5, MIN_THROTTLE + th * THROTTLE_RANGE / 256); break;
		default: break;
	}
	
	return;
}
