#include "user_spi.h"
#include "user_usart.h"
#include "nRF24L01.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
void SPI_NRF24_Setup(void);
void SPI_NRF24_CSN_Set(uint8_t logicValue);
void SPI_NRF24_CE_Set(uint8_t logicValue);
uint8_t SPI_NRF24_RW(uint8_t data);
uint8_t SPI_NRF24_WriteBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size);
uint8_t SPI_NRF24_ReadBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size);
uint8_t SPI_NRF24_ReadReg(uint8_t reg);
uint8_t SPI_NRF24_WriteReg(uint8_t reg, uint8_t data);*/

/*
const uint16_t NRF24_FIFO_SIZE =256;
uint8_t NRF24_rxfifo[NRF24_FIFO_SIZE] = {0x00, }
uint16_t NRF24_rxfifo_head = 0;
uint16_t NRF24_rxfifo_tail = 0;

void NRF24_rxfifo_push(uint8_t *data, uint8_t size){
	if 
}
*/

uint8_t ptx_addr[6] = "p0000"; // {0x34,0x43,0x10,0x10,0x01};			// Trans mode for send ACk to pilot
uint8_t prx_addr[6] = "p0000"; // {0x34,0x43,0x10,0x10,0x01}; // "p0000";			// Receive mode for getting command from Pilot

uint8_t rx_data[32];
uint8_t rx_dsize = 0;
uint8_t tx_data[32];
uint8_t tx_dsize = 0;
    
nRF24_state_t nRF24_state = {
	POWER_ON,						// mode
	ptx_addr,				// txaddr
	prx_addr,				// p0_rxaddr
	NULL, //(uint8_t*)"pilo1",				// p1_rxaddr
	// NULL, NULL, NULL, NULL, NULL,
	40,		// default channel 2
};

uint8_t nRF24_Check(void){
	uint8_t TX_ADDR_buf[NRF24_RA_TX_ADDR_BLEN] = {0xAA, 0x55, 0x01, 0x10, 0xFF};
	uint8_t TX_ADDR_buf_check[NRF24_RA_TX_ADDR_BLEN] = {0,};
	uint8_t i = 0;
	uint8_t ret = NRF24_SUCCESS;

	SPI_NRF24_WriteBuf( NRF24_W_REG_CMD(NRF24_RA_TX_ADDR), TX_ADDR_buf, NRF24_RA_TX_ADDR_BLEN);
	SPI_NRF24_ReadBuf( NRF24_R_REG_CMD(NRF24_RA_TX_ADDR), TX_ADDR_buf_check, NRF24_RA_TX_ADDR_BLEN);
	
	for (i = 0; i < NRF24_RA_TX_ADDR_BLEN; i ++){
		if (TX_ADDR_buf[i] != TX_ADDR_buf_check[i]){
			iored_printf("TX_ADDR Read/Write NON-Consistent %X ~ %X\r\n", TX_ADDR_buf[i], TX_ADDR_buf_check[i]);
			ret = NRF24_FAIL;
		}
	}

	return ret;
}

void nRF24_PowerUp(void){
	uint8_t config;
	nRF24_state.mode = POWER_UP;
	config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config |= 1 << NRF24_RA_CONFIG_MSK_PWR_UP_POS;
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);
	delay_msus(1, 500); // delay 1.5ms to go to standby-I mode
	nRF24_state.mode = STANDBY_I;
	//nRF24_FlushTX();
	//nRF24_FlushRX();
}

void nRF24_PowerDown(void){
	uint8_t config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config &= ~(1 << NRF24_RA_CONFIG_MSK_PWR_UP_POS);
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);
		SPI_NRF24_CE_Set(0);
	nRF24_state.mode = POWER_DOWN;
}

void nRF24_EnableInt(uint8_t msk){
	uint8_t config;
	config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config &= (~msk);
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);
}

void nRF24_DisableInt(uint8_t msk){
	uint8_t config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config |= msk;
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);
}

void nRF24_EnableCRC(void){
	uint8_t config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config |= (1 << NRF24_RA_CONFIG_MSK_EN_CRC_POS);
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);		
}

void nRF24_DisableCRC(void){
	uint8_t config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	config &= ~(1 << NRF24_RA_CONFIG_MSK_EN_CRC_POS);
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);		
}


void nRF24_SetPRX_TX(uint8_t bRX){
	uint8_t config = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	if (bRX){
		config |= 0x01;
	} else {
		config &= 0xFE;
	}
	
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, config);
}

void nRF24_EnableAA(uint8_t p){
	uint8_t aa = SPI_NRF24_ReadReg(NRF24_RA_EN_AA);
	aa |= (1 << p);
	SPI_NRF24_WriteReg(NRF24_RA_EN_AA, aa);
}

void nRF24_DisableAA(uint8_t p){
	uint8_t aa = SPI_NRF24_ReadReg(NRF24_RA_EN_AA);
	aa &= ~ (1<<p);
	SPI_NRF24_WriteReg(NRF24_RA_EN_AA, aa);
}

void nRF24_EnableRXAddr(uint8_t p){
	uint8_t aa = SPI_NRF24_ReadReg(NRF24_RA_EN_RXADDR);
	aa |= (1<<p);
	SPI_NRF24_WriteReg(NRF24_RA_EN_RXADDR, aa);	
}

void nRF24_DisableRXAddr(uint8_t p){
	uint8_t aa = SPI_NRF24_ReadReg(NRF24_RA_EN_RXADDR);
	aa &= ~(1<<p);
	SPI_NRF24_WriteReg(NRF24_RA_EN_RXADDR, aa);
}

void nRF24_SetAddrWidth(uint8_t w){
	uint8_t aa = 0;
	switch(w){
		case 3: aa = 0x01; break;
		case 4: aa = 0x02; break;
		case 5: aa = 0x03; break;
		default:
			return ;
	}
	
	SPI_NRF24_WriteReg(NRF24_RA_SETUP_AW, aa);
}

uint8_t nRF24_GetAddrWidth(void){
	return SPI_NRF24_ReadReg(NRF24_RA_SETUP_AW);
}

/*
void nRF24_SetRETR_i(uint8_t cfg){
	SPI_NRF24_WriteReg(NRF24_RA_SETUP_RETR, cfg);
}*/

void nRF24_SetRETR(uint8_t delay_i, uint8_t retry_i){
	uint8_t aa = ((delay_i & 0x0F) << 4) | (retry_i & 0x0F);
	SPI_NRF24_WriteReg(NRF24_RA_SETUP_RETR, aa);
}

uint8_t nRF24_GetRETR(void){
	return SPI_NRF24_ReadReg(NRF24_RA_SETUP_RETR);
}

void nRF24_SetRFCh(uint8_t ch){
	SPI_NRF24_WriteReg(NRF24_RA_RF_CH, ch & 0x7F);
}

uint8_t nRF24_GetRFCh(void){
	return SPI_NRF24_ReadReg(NRF24_RA_RF_CH);
}

void nRF24_SetupRF(uint8_t rf_setup){
	SPI_NRF24_WriteReg(NRF24_RA_RF_SETUP, rf_setup);
}

uint8_t nRF24_GetSetupRF(void){
		return SPI_NRF24_ReadReg(NRF24_RA_RF_SETUP);
}

uint8_t nRF24_GetCmdStatus(void){
	uint8_t status = 0;
	SPI_NRF24_CSN_Set(0);
	status = SPI_NRF24_RW(NRF24_SPI_CMD_NOP);
	SPI_NRF24_CSN_Set(1);
	return status;
}

uint8_t nRF24_GetStatus(void){
		return SPI_NRF24_ReadReg(NRF24_RA_STATUS);
}

void nRF24_ClearIntStatus(uint8_t mask){
	uint8_t st = nRF24_GetStatus();
	st |= mask;
	SPI_NRF24_WriteReg(NRF24_RA_STATUS, st);
}

uint8_t nRF24_GetPLossCnt(void){
	return (SPI_NRF24_ReadReg(NRF24_RA_OBSERVE_TX) & 0xF0) >> 4;
}

uint8_t nRF24_GetARCCnt(void){
	return SPI_NRF24_ReadReg(NRF24_RA_OBSERVE_TX) & 0x0F;
}

/* one or two bytes*/
void nRF24_SetCRCSchm(uint8_t schm){
	uint8_t cfg = SPI_NRF24_ReadReg(NRF24_RA_CONFIG);
	if (schm == 0){
		cfg &= ~(1<<NRF24_RA_CONFIG_MSK_CRCO_POS);
	} else {
		cfg |= (1 << NRF24_RA_CONFIG_MSK_CRCO_POS);
	}
	
	SPI_NRF24_WriteReg(NRF24_RA_CONFIG, cfg);
}

uint8_t nRF24_GetCRCSchm(void){
	return SPI_NRF24_ReadReg(NRF24_RA_CONFIG) & (1<<NRF24_RA_CONFIG_MSK_CRCO_POS);
}

void nRF24_SetPipeRXAddr(uint8_t p, uint8_t *rxaddr, uint8_t addrw){
	if (p == 0 || p == 1){
		SPI_NRF24_WriteBuf( NRF24_W_REG_CMD(NRF24_RA_RX_ADDR_P0 + p), rxaddr, addrw);
	} else if (p <= 5){
		SPI_NRF24_WriteReg(NRF24_RA_RX_ADDR_P0 + p, *rxaddr);
	}
}

void nRF24_GetPipeRXAddr(uint8_t p, uint8_t *rxaddr, uint8_t addrw){
	if (p == 0 || p == 1){
		SPI_NRF24_ReadBuf( NRF24_R_REG_CMD(NRF24_RA_RX_ADDR_P0 + p), rxaddr, addrw);
	} else if (p <= 5){
		*rxaddr = SPI_NRF24_ReadReg(NRF24_RA_RX_ADDR_P0 + p);
	}
}

void nRF24_SetTXAddr(uint8_t *txaddr, uint8_t addrw){
	SPI_NRF24_WriteBuf( NRF24_W_REG_CMD(NRF24_RA_TX_ADDR), txaddr, addrw);
}

void nRF24_GetTXAddr(uint8_t *txaddr, uint8_t addrw){
	SPI_NRF24_ReadBuf( NRF24_R_REG_CMD(NRF24_RA_TX_ADDR), txaddr, addrw);
}

void nRF24_SetPipePLoadWidth(uint8_t p, uint8_t width){
	if (p < 6 && width >= 1 && width <= 32){
		SPI_NRF24_WriteReg( NRF24_RA_RX_PW_P0 + p, width);
	}
}

uint8_t nRF24_GetPipePLoadWidth(uint8_t p){
	if (p < 6)
		return SPI_NRF24_ReadReg( NRF24_RA_RX_PW_P0 + p);
	else
		return 0;
}

void nRF24_FlushRX(void){
	SPI_NRF24_WriteBuf(NRF24_SPI_CMD_FLUSH_RX, NULL, 0);
}

void nRF24_FlushTX(void){
	SPI_NRF24_WriteBuf(NRF24_SPI_CMD_FLUSH_TX, NULL, 0);
}

uint8_t nRF24_GetFIFOStatus(void){
	return SPI_NRF24_ReadReg(NRF24_RA_FIFO_STATUS);
}

uint8_t nRF24_Config(void){
	int i = 0;
	if (nRF24_state.mode == POWER_DOWN || nRF24_state.mode == STANDBY_I){
		uint8_t dcheck[5] = {0x00, };
		// it's possible also in STANDBY_II mode, but difficult to control in software
		nRF24_SetAddrWidth(5);
		nRF24_SetRETR(4, 15); // Delay 500+86us, Retry for 5 times	
		//nRF24_SetTXAddr(nRF24_state.txaddr, 5);
		//nRF24_GetTXAddr(dcheck, 5);
		//if (memcmp(dcheck, nRF24_state.txaddr, 5) != 0){
		//	iored_printf("ERROR: TXADDR\r\n");
		//	return NRF24_FAIL;
		//}
		
		nRF24_SetPipeRXAddr(0, nRF24_state.p0_rxaddr, 5);
		nRF24_GetPipeRXAddr(0, dcheck, 5);
        for (i = 0; i < 5; i ++){
                iored_printf("%X ", dcheck[i]);
        }
				
		if (memcmp(dcheck, nRF24_state.p0_rxaddr, 5) != 0){
			iored_printf("ERROR: RXADDR0\r\n");
			return NRF24_FAIL;
		}
		
		/* PRX only receive from one PTX 
		nRF24_SetPipeRXAddr(1, nRF24_state.p1_rxaddr, 5);
		nRF24_GetPipeRXAddr(1, dcheck, 5);
		if (memcmp(dcheck, nRF24_state.p1_rxaddr, 5) != 0){
			iored_printf("ERROR: RXADDR1\r\n");
			return NRF24_FAIL;
		}*/
				
        SPI_NRF24_WriteReg(NRF24_RA_EN_AA, 0x01);
        SPI_NRF24_WriteReg(NRF24_RA_EN_RXADDR, 0x01);
        //nRF24_SetPipePLoadWidth(0, 4);
        /*nRF24_EnableAA(0);
        nRF24_DisableAA(1); nRF24_DisableAA(2); nRF24_DisableAA(3); 
        nRF24_DisableAA(4); nRF24_DisableAA(5);
        nRF24_EnableRXAddr(0);
		nRF24_DisableRXAddr(1); nRF24_DisableRXAddr(2);
        nRF24_DisableRXAddr(3); nRF24_DisableRXAddr(4);
        nRF24_DisableRXAddr(5); */
		nRF24_EnableInt(NRF24_INT_RX_DR | NRF24_INT_TX_DS | NRF24_INT_MAX_RT);
		nRF24_ClearIntStatus(NRF24_INT_ALL);
		nRF24_SetRFCh(nRF24_state.ch);		
				
        nRF24_EnableFeature(NRF24_FEATURE_DPL | NRF24_FEATURE_ACK_PAY |  NRF24_FEATURE_DYN_ACK);
        nRF24_EnablePipeDYNPD(0);
		nRF24_SetupRF(NRF24_RF_DR_2MBPS | NRF24_RF_PWR_0dBm | NRF24_LNA_GAIN);		
        nRF24_EnableCRC();
		nRF24_SetCRCSchm(1);
						
        //iored_printf("PLW = %X\r\n", nRF24_GetPipePLoadWidth(0));
		iored_printf("EN AA = %X\r\n", SPI_NRF24_ReadReg(NRF24_RA_EN_AA));
		iored_printf("EN RXAddr = %X\r\n", SPI_NRF24_ReadReg(NRF24_RA_EN_RXADDR));
		iored_printf("CH = %d\r\n", SPI_NRF24_ReadReg(NRF24_RA_RF_CH));
		iored_printf("RF= %X\r\n", nRF24_GetSetupRF());
		iored_printf("CONFIG = %X\r\n", SPI_NRF24_ReadReg(NRF24_RA_CONFIG));
				
		return NRF24_SUCCESS;
	} else {
		iored_printf("ERROR: nRF24_Config mode wrong\r\n");
		return NRF24_FAIL;
	}
}

void nRF24_EnableFeature(uint8_t f){
	SPI_NRF24_WriteReg(NRF24_RA_FEATURE, f);
}

void nRF24_EnablePipeDYNPD(uint8_t p){
	uint8_t cfg = SPI_NRF24_ReadReg(NRF24_RA_DYNPD);
	cfg |= 1 << (p + NRF24_RA_DYNPD_P0_POS);
	SPI_NRF24_WriteReg(NRF24_RA_DYNPD, cfg);
}


void nRF24_DisablePipeDYNPD(uint8_t p){
	uint8_t cfg = SPI_NRF24_ReadReg(NRF24_RA_DYNPD);
	cfg &= ~(1 << (p + NRF24_RA_DYNPD_P0_POS));
	SPI_NRF24_WriteReg(NRF24_RA_DYNPD, cfg);
}	

void nRF24_GotoRXMode(void){
	nRF24_SetPRX_TX(1);
	SPI_NRF24_CE_Set(1);
	delay_msus(0, 130);
	nRF24_state.mode = RX_MODE;
}

void nRF24_GotoStandbyI(void){
	SPI_NRF24_CE_Set(0);
	nRF24_state.mode = STANDBY_I;
}

// This will return the device to standby_i mode
void nRF24_GotoTXMode(void){
	nRF24_SetPRX_TX(0);
	SPI_NRF24_CE_Set(1);
	if (NRF24_IS_TXEMPTY (nRF24_GetFIFOStatus() ) ){
		nRF24_state.mode = STANDBY_II;
	} else {
		delay_msus(0, 15);
		SPI_NRF24_CE_Set(0);
		nRF24_state.mode = TX_MODE;
		delay_msus(0, 120);
	}
}

uint8_t nRF24_SendData(uint8_t *data, uint8_t size){
	if (nRF24_state.mode != STANDBY_I) {
		return NRF24_FAIL;
	}
		
	SPI_NRF24_WriteBuf(NRF24_SPI_CMD_W_TX_PLOAD, data, size);
	nRF24_GotoTXMode();
	return NRF24_SUCCESS;
}

void nRF24_GetDataFix(uint8_t *data, uint8_t size){
    SPI_NRF24_ReadBuf(NRF24_SPI_CMD_R_RX_PLOAD, data, size);
}

/* data size must >= 32 */
uint8_t nRF24_GetDataDyn(uint8_t *data){
	uint8_t rx_pw = 0;
	SPI_NRF24_ReadBuf(NRF24_R_RX_PL_WID, &rx_pw, 1);
	if (rx_pw > 32){
		nRF24_FlushRX();
		rx_pw = 0;
	}
	else
		SPI_NRF24_ReadBuf(NRF24_SPI_CMD_R_RX_PLOAD, data, rx_pw);
	return rx_pw;
}

void nRF24_SendACKData2Pipe(uint8_t p, uint8_t *data, uint8_t size){
	SPI_NRF24_WriteBuf(NRF24_W_ACK_PAYLOAD | (0x07 & p), data, size);
}

unsigned long nrf24_packet_last_ts;

void nRF24_exti_handler_cb(void){
	uint8_t i = 0;
	uint8_t int_status = nRF24_GetStatus();
	// Currently used in only PRX mode
	if (NRF24_IS_RX_DR(int_status) /* && NRF24_IS_TX_DS(int_status) */){
		rx_dsize = nRF24_GetDataDyn(rx_data);
		
        // Fixed length message
        /*nRF24_state.rx_dsize = 4;
        nRF24_GetDataFix(nRF24_state.rx_data, nRF24_state.rx_dsize);
        */
        // For debug
        /*iored_printf("RECV %d :", nRF24_state.rx_dsize);
        for (i = 0; i < nRF24_state.rx_dsize; i ++){
            iored_printf("%X ", nRF24_state.rx_data[i]);
        }
        iored_printf("\r\n");
        */
		//nRF24_ClearIntStatus(NRF24_INT_RX_DR /*| NRF24_INT_TX_DS*/);
		if (rx_dsize == 0){
			iored_printf("PACKET ERROR: Discarded\r\n");
		} else {
            //iored_printf("Sending ACK %d\r\n", nRF24_state.rx_dsize);
            nrf24_packet_handler(rx_data, rx_dsize, tx_data, &tx_dsize);
			nRF24_SendACKData2Pipe(0, tx_data, tx_dsize); // nRF24_state.rx_dsize);
			get_ms(&nrf24_packet_last_ts);
		}
	}
    
#ifdef NRF24_ACK_WITH_PAYLOAD
	if (NRF24_IS_TX_DS(int_status)){
		// ACK_PAY received
	}
#endif

    //if (NRF24_IS_TX_DS(int_status)) {
		// [TODO] This happens when device is used as PTX and a package is done
        // or when the device is PRX and a new packet from PTX is received
		//nRF24_ClearIntStatus(NRF24_INT_TX_DS);
		//nRF24_state.tx_data = NULL;
		//nRF24_state.mode = STANDBY_I;
	//}
	
	if (NRF24_IS_MAX_RT(int_status)) {
		// [TODO]
		//iored_printf("MAX INT\r\n");
		//nRF24_ClearIntStatus(NRF24_INT_MAX_RT);
		nRF24_GotoStandbyI();
		nRF24_FlushTX();
	}

	nRF24_ClearIntStatus(NRF24_INT_ALL);
}

int nrf24_is_connection_dead(unsigned long thre){
	unsigned long cur_ts = 0;
	get_ms(&cur_ts);
	return (cur_ts - nrf24_packet_last_ts > thre ? 1 : 0);
}
