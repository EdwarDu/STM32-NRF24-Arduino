#include "hc_sr04.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>

/** [TODO] The measurement of this module is very inaccurate,
 * Possible way is
 * 1. Use multiple sensors (more pins occupied)
 * 2. Measure multiple times, remove max, and min, and caculate the AVG (larger delay in code)
 */

#define HC_SR04_1_TRIG_PIN				GPIO_Pin_0
#define HC_SR04_1_ECHO_PIN				GPIO_Pin_1
#define HC_SR04_1_GPIO_PORT				GPIOC
#define HC_SR04_1_GPIO_RCC				RCC_APB2Periph_GPIOC

#define HC_SR04_2_TRIG_PIN				GPIO_Pin_0
#define HC_SR04_2_ECHO_PIN				GPIO_Pin_1
#define HC_SR04_2_GPIO_PORT				GPIOC

#define HC_SR04_3_TRIG_PIN				GPIO_Pin_0
#define HC_SR04_3_ECHO_PIN				GPIO_Pin_1
#define HC_SR04_3_GPIO_PORT				GPIOC

#define HC_SR04_4_TRIG_PIN				GPIO_Pin_0
#define HC_SR04_4_ECHO_PIN				GPIO_Pin_1
#define HC_SR04_4_GPIO_PORT				GPIOC

extern GPIO_InitTypeDef GPIO_InitStructure;
void hc_sr04_Setup(void){
	RCC_APB2PeriphClockCmd(HC_SR04_1_GPIO_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = HC_SR04_1_TRIG_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(HC_SR04_1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = HC_SR04_1_ECHO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(HC_SR04_1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(HC_SR04_1_GPIO_PORT, HC_SR04_1_TRIG_PIN);
}

extern unsigned long SysTick_ts_us;

const unsigned long dist_timeout_us = 10000;

float hc_sr04_Get_Dist(uint8_t ch){
	unsigned long start_ts = 0;
	unsigned long duration_us = 0;
	
	uint16_t hcsr04_pin_trig = 0;
	uint16_t hcsr04_pin_echo = 0;
	GPIO_TypeDef * hcsr04_port = NULL;
	
	switch(ch){
		case 1:
			hcsr04_pin_trig		= HC_SR04_1_TRIG_PIN;
			hcsr04_pin_echo		= HC_SR04_1_ECHO_PIN;
			hcsr04_port			= HC_SR04_1_GPIO_PORT;
			break;
		case 2:
			hcsr04_pin_trig		= HC_SR04_2_TRIG_PIN;
			hcsr04_pin_echo		= HC_SR04_2_ECHO_PIN;
			hcsr04_port			= HC_SR04_2_GPIO_PORT;
			break;
		case 3:
			hcsr04_pin_trig		= HC_SR04_3_TRIG_PIN;
			hcsr04_pin_echo		= HC_SR04_3_ECHO_PIN;
			hcsr04_port			= HC_SR04_3_GPIO_PORT;
			break;
		case 4:
			hcsr04_pin_trig		= HC_SR04_4_TRIG_PIN;
			hcsr04_pin_echo		= HC_SR04_4_ECHO_PIN;
			hcsr04_port			= HC_SR04_4_GPIO_PORT;
			break;
		default:
			return -1;
	}
	
	GPIO_SetBits(hcsr04_port, hcsr04_pin_trig);
	delay_msus(0, 10);
	GPIO_ResetBits(hcsr04_port, hcsr04_pin_trig);
	start_ts = SysTick_ts_us;
	while(0 == GPIO_ReadInputDataBit(hcsr04_port, hcsr04_pin_echo))
		if (SysTick_ts_us - start_ts > dist_timeout_us){
			return -2;
		}
	start_ts = SysTick_ts_us;
	while (1 == GPIO_ReadInputDataBit(hcsr04_port, hcsr04_pin_echo)) 
		if (SysTick_ts_us - start_ts > 5000) return -3;
	duration_us = SysTick_ts_us - start_ts;
	
	if (duration_us > 5000) return -3; // out of measure range
	return duration_us / 5.88f ;
}