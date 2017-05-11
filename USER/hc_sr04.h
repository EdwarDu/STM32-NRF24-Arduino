#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include "stm32f10x.h"

void hc_sr04_Setup(void);
float hc_sr04_Get_Dist(uint8_t ch);

#endif
