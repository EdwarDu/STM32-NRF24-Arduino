#ifndef _DELAY__H_
#define _DELAY__H_

#include "stm32f10x.h"
#include "misc.h"

uint8_t SysTick_Setup(void);
void delay_msus(uint32_t ms, uint16_t us);
void delay_ms(uint32_t ms);
void get_ms(unsigned long* timestamp);

#endif
