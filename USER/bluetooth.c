/**
 * [TODO] Bluetooth need to be used with DAM instead Interrupt 
 */

#include "bluetooth.h"
#include "delay.h"
#include <stdio.h>

extern USART_InitTypeDef USART_InitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;
extern DMA_InitTypeDef DMA_InitStructure;

#define BT_RECV_BUF_SIZE 					128
uint8_t bt_recv_buf[BT_RECV_BUF_SIZE];
uint8_t bt_recv_buf_datasize = 0;
/*uint16_t bt_recv_buf_head_index = 0;
uint16_t bt_recv_buf_tail_index = 0;
uint8_t bt_recv_buf_status = BT_RECV_BUF_STATUS_EMPTY_BIT;
*/

#define BT_SEND_BUF_SIZE					128
uint8_t bt_send_buf[BT_SEND_BUF_SIZE];
uint8_t bt_send_finished = 1;
/*uint16_t bt_send_buf_datasize = 0;
uint16_t bt_send_buf_head_index = 0;
uint16_t bt_send_buf_tail_index = 0;
uint8_t bt_send_buf_status = BT_SEND_BUF_STATUS_EMPTY_BIT;
*/

#define BT_PACKET_MIN_SIZE                  5
#define BT_PACKET_MAX_SIZE                  120
// A Packet must be transferred within 10ms
// With BaudRate = 115200 10 ms ~= 150Byte > BT_RECV(SEND)_BUF_SIZE
#define BT_PACKET_TIMEOUT_MS                10
// To keep the BT Alive, at leat one packet must be sent within 400ms
#define BT_CONN_TIMEOUT_MS                  400
#define BT_ACK_WRONG_CRC                    0xFF
#define BT_ACK_WRONG_EE                     0xFE
#define BT_ACK_DONE                         0x00
#define BT_ACK_DONE_WITH_PAYLOAD            0x01

#define BT_UART								USART3
#define BT_UART_IRQHANDLER					USART3_IRQHandler
#define BT_UART_RX_DMA_CH					DMA1_Channel3
#define BT_UART_RX_DMA_CH_FLAG				DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3
#define BT_UART_RX_DMA_CH_IRQN				DMA1_Channel3_IRQn
#define BT_UART_TX_DMA_CH					DMA1_Channel2
#define BT_UART_TX_DMA_CH_FLAG				DMA1_FLAG_GL2 | DMA2_FLAG_TC2 | DMA2_FLAG_TE2 | DMA2_FLAG_HT2
#define BT_UART_TX_DMA_CH_IRQN				DMA1_Channel2_IRQn


unsigned long bt_packet_head_ts, bt_packet_cur_ts;
unsigned long bt_packet_last_ts;

unsigned long bt_recv_size_prev = 0;

bt_packet_s cur_bt_packet = {
    0,
    0,
    bt_recv_buf, 
	bt_recv_buf+3};


// ack_payload
// payload_size : actual_payload

bt_packet_handler_cb_t bt_packet_handler_cb = NULL;

void bt_UART_Setup(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_DeInit(BT_UART);
	USART_InitStructure.USART_BaudRate = 9600; //115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(BT_UART, &USART_InitStructure);
	USART_ITConfig(BT_UART, USART_IT_IDLE, ENABLE);
	USART_ClearFlag(BT_UART, USART_FLAG_TC);
	USART_Cmd(BT_UART, ENABLE);
	
	// Bluetooth just as RF now
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = BT_UART_RX_DMA_CH_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = BT_UART_TX_DMA_CH_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// DMA
	DMA_DeInit(BT_UART_RX_DMA_CH);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &BT_UART->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) bt_recv_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = BT_RECV_BUF_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(BT_UART_RX_DMA_CH, &DMA_InitStructure);
	
	USART_DMACmd(BT_UART, USART_DMAReq_Rx, ENABLE);
	DMA_ITConfig(BT_UART_RX_DMA_CH, DMA_IT_TC, ENABLE);
	
	DMA_ClearFlag(BT_UART_RX_DMA_CH_FLAG); // Clear the Flags
	DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
	
	DMA_DeInit(BT_UART_TX_DMA_CH);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & BT_UART->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) bt_send_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = BT_SEND_BUF_SIZE;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(BT_UART_TX_DMA_CH, &DMA_InitStructure);
	
	USART_DMACmd(BT_UART, USART_DMAReq_Tx, ENABLE);

	DMA_ITConfig(BT_UART_TX_DMA_CH, DMA_IT_TC, ENABLE);
	
	DMA_ClearFlag(BT_UART_TX_DMA_CH_FLAG);
	DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 0);
	DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
}

void bt_recv_buf_Reset(void){
    DMA_Cmd(BT_UART_RX_DMA_CH, DISABLE);        // Stop Recv Now
    DMA_ClearFlag(BT_UART_RX_DMA_CH_FLAG); // Clear the Flags
    DMA_SetCurrDataCounter(BT_UART_RX_DMA_CH, BT_RECV_BUF_SIZE);
    DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
}

void bt_send_buf_Reset(void){
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);        // Stop Send Now
    DMA_ClearFlag(BT_UART_TX_DMA_CH_FLAG); // Clear the Flags
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 0);
    //DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_SendCRCWrongACK(void){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
    bt_send_buf[0] = BT_ACK_HEADER;
    bt_send_buf[1] = cur_bt_packet.id;
    bt_send_buf[2] = 0x01;
    bt_send_buf[3] = BT_ACK_WRONG_CRC;
    bt_send_buf[4] = BT_ACK_TAIL;
    //bt_send_buf[5] = bt_send_buf[0] ^ bt_send_buf[1] ^ bt_send_buf[2] ^
    //                bt_send_buf[3] ^ bt_send_buf[4];
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 5);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_SendWrongEEACK(void){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
    bt_send_buf[0] = BT_ACK_HEADER;
    bt_send_buf[1] = cur_bt_packet.id;
    bt_send_buf[2] = 0x01;
    bt_send_buf[3] = BT_ACK_WRONG_EE;
    bt_send_buf[4] = BT_ACK_TAIL;
    //bt_send_buf[5] = bt_send_buf[0] ^ bt_send_buf[1] ^ bt_send_buf[2] ^
    //                bt_send_buf[3] ^ bt_send_buf[4];
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 5);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_SendDoneACK(uint8_t res){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
    bt_send_buf[0] = BT_ACK_HEADER;
    bt_send_buf[1] = cur_bt_packet.id;
    bt_send_buf[2] = 0x02;
    bt_send_buf[3] = BT_ACK_DONE;
    bt_send_buf[4] = res;
    bt_send_buf[5] = BT_ACK_TAIL;
    //bt_send_buf[6] = bt_send_buf[0] ^ bt_send_buf[1] ^ bt_send_buf[2] ^
     //               bt_send_buf[3] ^ bt_send_buf[4] ^ bt_send_buf[5];
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 6);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_t1(void){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
     bt_send_buf[0] = 'A';
    bt_send_buf[1] = 'T';
    bt_send_buf[2] = '+';
    bt_send_buf[3] = 'B';
    bt_send_buf[4] = 'A';
    bt_send_buf[5] = 'U';
    bt_send_buf[6] = 'D';
	bt_send_buf[7] = '4';
    //bt_send_buf[6] = bt_send_buf[0] ^ bt_send_buf[1] ^ bt_send_buf[2] ^
     //               bt_send_buf[3] ^ bt_send_buf[4] ^ bt_send_buf[5];
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 8);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_t2(void){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
     bt_send_buf[0] = 'A';
    bt_send_buf[1] = 'T';
    //bt_send_buf[6] = bt_send_buf[0] ^ bt_send_buf[1] ^ bt_send_buf[2] ^
     //               bt_send_buf[3] ^ bt_send_buf[4] ^ bt_send_buf[5];
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, 2);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void bt_SendACKWithPayload_Init(void){
    while(!bt_send_finished);
    DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
}

void bt_SendACKWithPayload_Send(void){
    bt_send_buf[0] = BT_ACK_HEADER;
    bt_send_buf[1] = cur_bt_packet.id;
    bt_send_buf[bt_send_buf[2] + 3] = BT_ACK_TAIL;
    DMA_SetCurrDataCounter(BT_UART_TX_DMA_CH, bt_send_buf[2]+4);
    bt_send_finished = 0;
    DMA_Cmd(BT_UART_TX_DMA_CH, ENABLE);
}

void BT_UART_IRQHANDLER(void){
	uint16_t i = 0, j = 0;
	
	if (USART_GetITStatus(BT_UART, USART_IT_IDLE) != RESET){
		//DMA_Cmd(BT_UART_RX_DMA_CH, DISABLE);		// Stop Recv Now
		bt_recv_buf_datasize = BT_RECV_BUF_SIZE - DMA_GetCurrDataCounter(BT_UART_RX_DMA_CH);
        
		if (bt_recv_buf_datasize != bt_recv_size_prev){
			get_ms(&bt_packet_head_ts);
            bt_recv_size_prev = bt_recv_buf_datasize;
        }
        
        if (bt_recv_buf_datasize < BT_PACKET_MIN_SIZE){
            // DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
        } else if (bt_recv_buf[0] != BT_P_HEADER){
            // current (incomplete) package is wrong
			DMA_Cmd(BT_UART_RX_DMA_CH, DISABLE);
            for (i = 1; i < bt_recv_buf_datasize; i ++){
                if (bt_recv_buf[i] == BT_P_HEADER) break;
            }
            
            bt_recv_buf_datasize = bt_recv_buf_datasize - i;
            bt_recv_size_prev = bt_recv_buf_datasize;

            if (i != bt_recv_buf_datasize){
                for (j = 0; j < bt_recv_buf_datasize; i ++, j ++){
                    bt_recv_buf[j] = bt_recv_buf[i]; // move the packet
                }
            }
            
            // can't find packet header //reload DMA
            DMA_ClearFlag(BT_UART_RX_DMA_CH_FLAG); // Clear the Flags
            BT_UART_RX_DMA_CH->CNDTR = BT_RECV_BUF_SIZE - bt_recv_buf_datasize;
            DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
        } else {
            cur_bt_packet.id = bt_recv_buf[1];
            cur_bt_packet.payload_size = bt_recv_buf[2];
            
            if (bt_recv_buf_datasize < cur_bt_packet.payload_size + 5){
                // if 10ms passed since last we got data, and packet still not complete
                // We act as timed out
                get_ms(&bt_packet_cur_ts);
                if (bt_packet_cur_ts - bt_packet_head_ts > BT_PACKET_TIMEOUT_MS){
                    bt_recv_buf_Reset();
                    bt_recv_size_prev = 0;
                } //else DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
            } else if (bt_recv_buf[cur_bt_packet.payload_size + 3] != BT_P_TAIL){ // Check the packet ending
				bt_recv_buf_Reset();
				bt_SendWrongEEACK();
                bt_recv_size_prev = 0;
            } else {
                uint8_t crc = bt_recv_buf[cur_bt_packet.payload_size + 4];
                bt_recv_buf_datasize = bt_recv_buf_datasize - cur_bt_packet.payload_size - 5;
                
                for (i = 0 ; i < cur_bt_packet.payload_size + 4; i ++){
                    crc ^= bt_recv_buf[i];
                }
                
                if (crc != 0){ // broken packet
                    bt_SendCRCWrongACK();
                } else {
                    get_ms(&bt_packet_last_ts);
                    if (bt_packet_handler_cb != NULL && cur_bt_packet.payload_size != 0){
                        i = bt_packet_handler_cb(bt_send_buf+2);
                        if (bt_send_buf[2] == 0) {
                            bt_SendDoneACK(i);
                        }
                    } else {
                        bt_SendDoneACK(0xFF);
                    }
                }
                
				// Only one packet at a time, no need to disable and enable again
				// DMA_SetCurrDataCounter(BT_UART_RX_DMA_CH, BT_RECV_BUF_SIZE);
				bt_recv_buf_Reset();
				bt_recv_size_prev = 0;
            }
        }
        
        // Clear IDLE Status
		//USART_ClearFlag(BT_UART, USART_FLAG_IDLE);
		i = BT_UART->SR;
		i = BT_UART->DR;
	}
	
	if(USART_GetITStatus(BT_UART, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET) { // ERROR
		USART_ClearITPendingBit(BT_UART, USART_IT_PE | USART_IT_FE | USART_IT_NE);
        bt_recv_buf_Reset();
	}
	
	//USART_ClearITPendingBit(BT_UART, USART_IT_TC);
	USART_ClearITPendingBit(BT_UART, USART_IT_IDLE);
}

void DMA1_Channel3_IRQHandler(void){
	if (DMA_GetITStatus(DMA1_IT_TC3)){
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		DMA_Cmd(BT_UART_RX_DMA_CH, DISABLE);
		BT_UART_RX_DMA_CH->CNDTR = BT_RECV_BUF_SIZE;
		DMA_Cmd(BT_UART_RX_DMA_CH, ENABLE);
	}
}

void DMA1_Channel2_IRQHandler(void){
	if (DMA_GetITStatus(DMA1_IT_TC2)){
		DMA_ClearITPendingBit(DMA1_IT_TC2);
		DMA_Cmd(BT_UART_TX_DMA_CH, DISABLE);
		bt_send_finished=1;
	}
}

void bt_Register_PacketCB(bt_packet_handler_cb_t cb){
    bt_packet_handler_cb = cb;
}

void bt_DeRegister_PacketCB(void){
    bt_packet_handler_cb = NULL;
}

int bt_isConnectionDead(void){
    unsigned long cur_ts = 0;
    get_ms(&cur_ts);
    if (cur_ts - bt_packet_last_ts > BT_CONN_TIMEOUT_MS){
        return 1;       // Dead
    } else {
        return 0;       // BT Connection Alive
    }
}
/*
void UART4_IRQHandler(void){
 	if (USART_GetITStatus(BT_UART, USART_IT_RXNE) != RESET){
		if (bt_recv_buf_status & BT_RECV_BUF_STATUS_FULL_BIT){
			bt_recv_buf_status |= BT_RECV_BUF_STATUS_OF_BIT;
			printf("Overflow!!!!!!!!\r\n");
			return;
		}

		if (bt_recv_buf_datasize == BT_RECV_BUF_SIZE){
			bt_recv_buf_status |= BT_RECV_BUF_STATUS_FULL_BIT;
		}

		bt_recv_buf[bt_recv_buf_tail_index++] = USART_ReceiveData(BT_UART);
		bt_recv_buf_tail_index %= BT_RECV_BUF_SIZE;
		bt_recv_buf_datasize ++;
		bt_recv_buf_status &= ~BT_RECV_BUF_STATUS_EMPTY_BIT;
	}

	if (USART_GetITStatus(BT_UART, USART_IT_TC) != RESET){
		if (bt_send_buf_datasize != 0){
			USART_SendData(BT_UART, bt_send_buf[bt_send_buf_head_index++]);
			bt_send_buf_head_index %= BT_SEND_BUF_SIZE;
			if (! --bt_send_buf_datasize){
				bt_send_buf_status |= BT_SEND_BUF_STATUS_EMPTY_BIT;
			}
		} else {
			bt_send_buf_status |= BT_SEND_BUF_STATUS_EMPTY_BIT;
			USART_ClearITPendingBit(BT_UART, USART_IT_TC);
		}
	}
}

void bt_UART_SendData(uint8_t *str, uint16_t size){
	uint16_t i = 0;
	for (i = 0; i < size; i ++){
		USART_SendData(BT_UART, str[i]);
		while (RESET == USART_GetFlagStatus(BT_UART, USART_FLAG_TC));
	}
}

uint8_t bt_SendData(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	uint8_t write_byte =  size < (BT_SEND_BUF_SIZE - bt_send_buf_datasize) ? 
								size : (BT_SEND_BUF_SIZE - bt_send_buf_datasize);

	if (write_byte == 0) return 0; // FULL

	for (i = 0; i < write_byte; i ++){
		bt_send_buf[bt_send_buf_tail_index++] = data[i];
		bt_send_buf_tail_index %= BT_SEND_BUF_SIZE;
	}

	bt_send_buf_datasize += write_byte - 1; // reason for -1 is one byte write must be triggered here
	// Trigger the first send
	USART_SendData(BT_UART, bt_send_buf[bt_send_buf_head_index++]);
	bt_send_buf_head_index %= BT_SEND_BUF_SIZE;
	bt_send_buf_status &= ~ BT_SEND_BUF_STATUS_EMPTY_BIT;

	return write_byte;
}

void bt_SendData_Block(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	while (i < size){
		i += bt_SendData(data+i, size-i);
	}
}

uint8_t bt_GetData(uint8_t *data, uint8_t size){
	uint8_t i = 0;
	uint8_t read_byte = bt_recv_buf_datasize < size ? bt_recv_buf_datasize : size;

	if (read_byte == 0) return 0;
	
	for (i = 0; i < read_byte; i ++){
		data[i] = bt_recv_buf[bt_recv_buf_head_index++];
		bt_recv_buf_head_index %= BT_RECV_BUF_SIZE;
	}

	bt_recv_buf_datasize -= read_byte;
	if (bt_recv_buf_datasize == 0){
		bt_recv_buf_status |= BT_RECV_BUF_STATUS_EMPTY_BIT;
	}

	return read_byte;
} */
