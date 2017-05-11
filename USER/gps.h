#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "stm32f10x.h"

#define BT_AT_BAUD_9600					0		/*Default*/
#define BT_AT_BAUD_19200				1
#define BT_AT_BAUD_38400				2
#define BT_AT_BAUD_57600				3
#define BT_AT_BAUD_115200				4

#define BT_AT_PARI_NONE					0		/*Default*/
#define BT_AT_PARI_EVEN					1
#define BT_AT_PARI_ODD					2

#define BT_AT_STOP_1					0		/*Default*/
#define BT_AT_STOP_2					1

#define BT_AT_MODE_PT					0 		/*Default : Pass Through*/
#define BT_AT_MODE_RC					1 		/* Remote Control */
#define BT_AT_MODE_COMB					2

#define BT_AT_PIO0_0					0 		/*Default*/
#define BT_AT_PIO0_1					1

#define BT_AT_PIO1_BLINK				0 		/*Default*/
#define BT_AT_PIO1_NO					1

#define BT_AT_ROLE_MASTER				0 		/*Default*/
#define BT_AT_ROLE_SLAVE				1

#define BT_AT_POWE_N23					0
#define BT_AT_POWE_N6					1
#define BT_AT_POWE_ZERO					2 		/*Default*/
#define BT_AT_POWE_6 					3

void bt_UART_Setup(void);

void bt_UART_SendData(uint8_t *str, uint16_t size);

#endif
