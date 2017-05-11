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


#define BT_P_HEADER     0xBB
#define BT_P_TAIL       0xEE

#define BT_ACK_HEADER   0xBA
#define BT_ACK_TAIL     0xAE

void bt_UART_Setup(void);

typedef struct {
    uint8_t id;
    uint8_t payload_size;
    uint8_t *packet;
	uint8_t *payload;
} bt_packet_s;

typedef uint8_t (*bt_packet_handler_cb_t) (uint8_t *ack_payload);
// ack_payload
// payload_size : actual_payload
// payload_size == 0 will cause a DONE ACK with return value sent back

void bt_recv_buf_Reset(void);
void bt_send_buf_Reset(void);
void bt_SendACKWithPayload_Init(void);
void bt_SendACKWithPayload_Send(void);
void bt_Register_PacketCB(bt_packet_handler_cb_t cb);
void bt_DeRegister_PacketCB(void);
int bt_isConnectionDead(void);
void bt_SendCRCWrongACK(void);
void bt_SendDoneACK(uint8_t res);
void bt_t1(void);
void bt_t2(void);

/*
void bt_UART_SendData(uint8_t *str, uint16_t size);
uint8_t bt_GetData(uint8_t *data, uint8_t size);
uint8_t bt_SendData(uint8_t *data, uint8_t size);
void bt_SendData_Block(uint8_t *data, uint8_t size);*/


#endif
