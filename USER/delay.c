#include "stm32f10x.h"
#include "delay.h"

unsigned long SysTick_us_Counter = 0;

uint8_t SysTick_Setup(void){
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	if (SysTick_Config(SystemCoreClock / 1000000)){ // Tick every 1us
		return 1;
	}
	
	NVIC_SetPriority(SysTick_IRQn, 0);
	// SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk; // Enable from the call of this function
	return 0;
}

void delay_msus(uint32_t ms, uint16_t us){
	SysTick_us_Counter = ms * 1000 + us;
	// SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	while (SysTick_us_Counter != 0) ;
	// SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}


void delay_ms(uint32_t ms){
	delay_msus(ms, 0);
}

extern unsigned long SysTick_ts_us;
void get_ms(unsigned long* timestamp){
    *timestamp = SysTick_ts_us / 1000;
}