#ifndef __USER_USART_H__
#define __USER_USART_H__

#include "stm32f10x.h"

#define USE_IO_RED

#ifdef USE_IO_RED
	#define iored_printf printf
    #define iored_scanf scanf
#else
	#define iored_printf(...)
    #define iored_scanf(...)
#endif

#define IO_REDIR_USART_PORT			USART1

void USART_IO_ReD_Setup(void);

#endif // __USER_USART_H__
