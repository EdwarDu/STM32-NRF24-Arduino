#ifndef __USER_PWM_H__
#define __USER_PWM_H__

#include "stm32f10x.h"
void TIM5_PWM_Setup(void);
void TIM5_PWM_Set_Throttle(uint8_t ch, uint8_t th);

#endif
