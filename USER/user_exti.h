#ifndef __USER_EXTI_H__
#define __USER_EXTI_H__

#include "stm32f10x.h"

typedef void (*exti_handler_cb_t)(void);

void EXTI_Setup(void);
void EXTI_IRQnHanlder_DeRegisterCB(uint32_t EXTI_Line);
void EXTI_IRQnHanlder_RegisterCB(uint32_t EXTI_Line, exti_handler_cb_t cb);

#endif //__USER_EXTI_H__
