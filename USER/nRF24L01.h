#ifndef __NRF24L01_H__
#define __NRF24L01_H__
#include "user_spi.h"

/* User Definition */
#define NRF24_SUCCESS		0x00
#define NRF24_FAIL			0x01

#define NRF24_ACK_WITH_PAYLOAD
#define NRF24_ROLE_PRX_ONLY

/* Functions */
uint8_t nRF24_Check(void);
uint8_t nRF24_GetStatus(void);
void nRF24_PowerUp(void);
void nRF24_PowerDown(void);
uint8_t nRF24_Config(void);
void nRF24_GotoRXMode(void);
void nRF24_FlushTX(void);
void nRF24_FlushRX(void);
void nRF24_SetCRCSchm(uint8_t schm);
uint8_t nRF24_GetCRCSchm(void);
void nRF24_DisablePipeDYNPD(uint8_t p);
void nRF24_EnablePipeDYNPD(uint8_t p);
void nRF24_EnableFeature(uint8_t f);

void nRF24_exti_handler_cb(void);
uint8_t nRF24_SendData(uint8_t *data, uint8_t size);
uint8_t nRF24_GetPLossCnt(void);
uint8_t nRF24_GetARCCnt(void);
uint8_t nRF24_GetRFCh(void);
void nRF24_SetRFCh(uint8_t ch);
void nRF24_GetPipeRXAddr(uint8_t p, uint8_t *rxaddr, uint8_t addrw);
void nRF24_GetTXAddr(uint8_t *txaddr, uint8_t addrw);
uint8_t nRF24_GetFIFOStatus(void);
uint8_t nRF24_GetStatus(void);
void nRF24_EnableCRC(void);
void nRF24_DisableCRC(void);
uint8_t nRF24_GetSetupRF(void);
int nrf24_is_connection_dead(unsigned long thre);
uint8_t nRF24_GetPipePLoadWidth(uint8_t p);

/*---------------------------NRF24 Packet Handler Sig-----------------------*/
void nrf24_packet_handler(uint8_t *packet, uint8_t size, uint8_t *ack, uint8_t *pACKsize);

typedef enum {
	POWER_OFF,
	POWER_ON,
	POWER_DOWN,
	POWER_UP,
	STANDBY_I,
	STANDBY_II,
	RX_MODE,
	TX_MODE } nRF24_mode_t;

typedef struct {
	nRF24_mode_t mode;
	uint8_t *txaddr;
	uint8_t *p0_rxaddr;
	uint8_t *p1_rxaddr; // , p1_rxaddr, p2_rxaddr, p3_rxaddr, p4_rxaddr, p5_rxaddr;
	uint8_t ch;
} nRF24_state_t;

extern nRF24_state_t nRF24_state; 

/* nRF24L01 Register Map */
#define NRF24_RA_CONFIG							0x00
#define NRF24_RA_CONFIG_MSK_RX_DR_POS			6
#define NRF24_RA_CONFIG_MSK_TX_DS_POS			5
#define NRF24_RA_CONFIG_MSK_MAX_RT_POS			4
#define NRF24_RA_CONFIG_MSK_EN_CRC_POS			3
#define NRF24_RA_CONFIG_MSK_CRCO_POS			2
#define NRF24_RA_CONFIG_MSK_PWR_UP_POS			1
#define NRF24_RA_CONFIG_MSK_PRIM_RX_POS			0

#define NRF24_RA_EN_AA							0x01
#define NRF24_RA_EN_AA_P5 						5
#define NRF24_RA_EN_AA_P4 						4
#define NRF24_RA_EN_AA_P3 						3
#define NRF24_RA_EN_AA_P2 						2
#define NRF24_RA_EN_AA_P1 						1
#define NRF24_RA_EN_AA_P0 						0

#define NRF24_RA_EN_RXADDR						0x02
#define NRF24_RA_EN_RXADDR_P5					5
#define NRF24_RA_EN_RXADDR_P4					4
#define NRF24_RA_EN_RXADDR_P3					3
#define NRF24_RA_EN_RXADDR_P2					2
#define NRF24_RA_EN_RXADDR_P1					1
#define NRF24_RA_EN_RXADDR_P0					0

#define NRF24_RA_SETUP_AW						0x03
#define NRF24_RA_SETUP_AW_POS					0
#define NRF24_RA_SETUP_AW_LEN					2
/* SETUP AW
 * 00 - Illegal
 * 01 - 3Bytes
 * 10 - 4Bytes
 * 11 - 5Bytes*/

#define NRF24_RA_SETUP_RETR						0x04
#define NRF24_RA_SETUP_RETR_ARD_POS				4
#define NRF24_RA_SETUP_RETR_ARD_LEN				4
/* Atuo Re-transmit Delay
 * 0000 - Wait 250 + 86us
 * 0001 - Wait 500 + 86us
 * 0010 - Wait 750 + 86us
 * ....
 * 1111 - Wait 4000 + 86us */
#define NRF24_RA_SETUP_RETR_ARC_POS				0
#define NRF24_RA_SETUP_RETR_ARC_LEN				4
/* Auto Retransmit Count
 * 0000 - Re-Transmit Disabled
 * 0001 - Up to 1 Re-Transmit of AA
 * ....
 * 1111 - Up to 15 ..... */

#define NRF24_RA_RF_CH							0x05
#define NRF24_RA_RF_CH_POS						0
#define NRF24_RA_RF_CH_LEN						7

#define NRF24_RA_RF_SETUP						0x06
#define NRF24_RA_RF_SETUP_CONT_WAVE_POS         7
#define NRF24_RA_RF_SETUP_RF_DR_LOW_POS         5
#define NRF24_RA_RF_SETUP_PLL_LCK_POS			4
#define NRF24_PLL_LCK							0x10
#define NRF24_PLL_NOLCK							0x00
#define NRF24_RA_RF_SETUP_RF_DR_HIGH_POS        3
/* Data Rate
 * LOW  HIGH
 * 00 - 1Mbps
 * 01 - 2Mbps
 * 10 - 250kbps */
#define NRF24_RF_DR_1MBPS						0x00
#define NRF24_RF_DR_2MBPS						0x08
#define NRF24_RF_DR_250KBPS						0x20
#define NRF24_RA_RF_SETUP_RF_PWR_POS			1
#define NRF24_RA_RF_SETUP_RF_PWR_LEN 			2
/* RF output power in TX Mode
 * 00 - -18 dBm
 * 01 - -12 dBm
 * 10 - -6 dBm
 * 11 - 0 dBm */
#define NRF24_RF_PWR_N18dBm						0x00
#define NRF24_RF_PWR_N12dBm						0x02
#define NRF24_RF_PWR_N6dBm						0x04
#define NRF24_RF_PWR_0dBm						0x06
#define NRF24_RA_RF_SETUP_LNA_HCURR_POS			0
/* Setup LNA gain */
#define NRF24_LNA_GAIN							0x01
#define NRF24_LNA_NOGAIN						0x00

#define NRF24_RA_STATUS							0x07
#define NRF24_RA_STATUS_RX_DR_POS				6
#define NRF24_RA_STATUS_TX_DS_POS				5
#define NRF24_RA_STATUS_MAX_RT_POS				4
#define NRF24_RA_STATUS_RX_P_NO_POS				1
#define NRF24_RA_STATUS_RX_P_NO_LEN				3
#define NRF24_RA_STATUS_TX_FULL_POS				0

#define NRF24_INT_RX_DR							0x40
#define NRF24_INT_TX_DS							0x20
#define NRF24_INT_MAX_RT						0x10
#define NRF24_INT_ALL							0x70

/* TX FIFO full flag
 * 1 - TX FIFO Full 
 * 0 - Available locations in TX FIFO */
#define NRF24_IS_RX_DR(x) (x & (1<<NRF24_RA_STATUS_RX_DR_POS))
#define NRF24_IS_TX_DS(x) (x & (1<<NRF24_RA_STATUS_TX_DS_POS))
#define NRF24_IS_MAX_RT(x) (x & (1<<NRF24_RA_STATUS_MAX_RT_POS))
#define NRF24_GET_RX_P_NO(x) ((x>>NRF24_RA_STATUS_RX_P_NO_POS) & ((1<< (NRF24_RA_STATUS_RX_P_NO_LEN+1)) -1))
#define NRF24_IS_TX_FULL(x) (x & (1<<NRF24_RA_STATUS_TX_FULL_POS))


#define NRF24_RA_OBSERVE_TX						0x08
#define NRF24_RA_OBSERVE_TX_PLOS_CNT_POS		4
#define NRF24_RA_OBSERVE_TX_PLOS_CNT_LEN		4
/* Packet Loss Counter (ReadOnly) */
#define NRF24_RA_OBSERVE_TX_ARC_CNT_POS			0
#define NRF24_RA_OBSERVE_TX_ARC_CNT_LEN			4
/* Auto Resent Counter (ReadOnly) */

#define NRF24_RA_CD								0x09   /* Carrier Detect ReadOnly*/
#define NRF24_RA_CD_POS							0

#define NRF24_RA_RX_ADDR_P0						0x0A
#define NRF24_RA_RX_ADDR_P0_BLEN				5

#define NRF24_RA_RX_ADDR_P1						0x0B
#define NRF24_RA_RX_ADDR_P1_BLEN				5

#define NRF24_RA_RX_ADDR_P2						0x0C
#define NRF24_RA_RX_ADDR_P3						0x0D
#define NRF24_RA_RX_ADDR_P4						0x0E
#define NRF24_RA_RX_ADDR_P5						0x0F

#define NRF24_RA_TX_ADDR 						0x10
#define NRF24_RA_TX_ADDR_BLEN					5

#define NRF24_RA_RX_PW_P0						0x11
#define NRF24_RA_RX_PW_P0_POS					0
#define NRF24_RA_RX_PW_P0_LEN					6
/* Number of bytes in RX payload in data pipe 0
 * 0  - Illegal
 * 1  - 1Byte
 * 2  - 2Byte */

#define NRF24_RA_RX_PW_P1						0x12
#define NRF24_RA_RX_PW_P1_POS					0
#define NRF24_RA_RX_PW_P1_LEN					6
#define NRF24_RA_RX_PW_P2						0x13
#define NRF24_RA_RX_PW_P2_POS					0
#define NRF24_RA_RX_PW_P2_LEN					6
#define NRF24_RA_RX_PW_P3						0x14
#define NRF24_RA_RX_PW_P3_POS					0
#define NRF24_RA_RX_PW_P3_LEN					6
#define NRF24_RA_RX_PW_P4						0x15
#define NRF24_RA_RX_PW_P4_POS					0
#define NRF24_RA_RX_PW_P4_LEN					6
#define NRF24_RA_RX_PW_P5						0x16
#define NRF24_RA_RX_PW_P5_POS					0
#define NRF24_RA_RX_PW_P5_LEN					6

#define NRF24_RA_FIFO_STATUS 					0x17
#define NRF24_RA_FIFO_STATUS_TX_REUSE_POS		6
/* Reuse last send data packet */
#define NRF24_RA_FIFO_STATUS_TX_FULL_POS		5
#define NRF24_RA_FIFO_STATUS_TX_EMPTY_POS		4
#define NRF24_RA_FIFO_STATUS_RX_FULL_POS		1
#define NRF24_RA_FIFO_STATUS_RX_EMPTY_POS		0

#define NRF24_IS_TXREUSE(x) (x & (1<<NRF24_RA_FIFO_STATUS_TX_REUSE_POS))
#define NRF24_IS_TXFULL(x) (x & (1<<NRF24_RA_FIFO_STATUS_TX_FULL_POS))
#define NRF24_IS_TXEMPTY(x) (x & (1<<NRF24_RA_FIFO_STATUS_TX_EMPTY_POS))
#define NRF24_IS_RXFULL(x) (x & (1<<NRF24_RA_FIFO_STATUS_RX_FULL_POS))
#define NRF24_IS_RXEMPTY(x) (x & (1<<NRF24_RA_FIFO_STATUS_RX_EMPTY_POS))

#define NRF24_RA_DYNPD							0x1C
#define NRF24_RA_DYNPD_P5_POS					5
#define NRF24_RA_DYNPD_P4_POS					4
#define NRF24_RA_DYNPD_P3_POS					3
#define NRF24_RA_DYNPD_P2_POS					2
#define NRF24_RA_DYNPD_P1_POS					1
#define NRF24_RA_DYNPD_P0_POS					0

#define NRF24_RA_FEATURE						0x1D
#define NRF24_RA_FEATURE_EN_DPL    				2
#define NRF24_RA_FEATURE_EN_ACK_PAY				1
#define NRF24_RA_FEATURE_EN_DYN_ACK				0

#define NRF24_FEATURE_DPL						0x04
#define NRF24_FEATURE_ACK_PAY					0x02
#define NRF24_FEATURE_DYN_ACK					0x01

#define NRF24_SPI_CMD_R_REG						0x00
#define NRF24_SPI_CMD_W_REG						0x20   /* Executable in power down or standby modes only */
#define NRF24_SPI_CMD_RW_REG_MSK				0x1F
#define NRF24_SPI_CMD_R_RX_PLOAD				0x61
#define NRF24_SPI_CMD_W_TX_PLOAD				0xA0
#define NRF24_SPI_CMD_FLUSH_TX					0xE1   /* Used in TX Mode */
#define NRF24_SPI_CMD_FLUSH_RX					0xE2   /* Used in RX mode, should not be executed during transmission of ACK */
#define NRF24_SPI_CMD_REUSE_TX_PL				0xE3   /* Used for a PTX device, reuse last sent payload. Packets will be repeatedly resent as long as CE is high
														TX payload reuse ia active untill W_TX_PAYLOAD or FLUSH TX is executed */
#define NRF24_R_RX_PL_WID						0x60
#define NRF24_W_ACK_PAYLOAD						0xA8
#define NRF24_W_TX_PAYLOAD_NO_ACK				0xB0
#define NRF24_SPI_CMD_NOP						0xFF   /* NOP. Might be used to read the STATUS register */

// Two macros for composing the read/write regsiter SPI command
#define NRF24_R_REG_CMD(x)	(NRF24_SPI_CMD_RW_REG_MSK & x) // (NRF24_SPI_CMD_R_REG | (NRF24_SPI_CMD_RW_REG_MSK & x))
#define NRF24_W_REG_CMD(x)	(NRF24_SPI_CMD_W_REG | (NRF24_SPI_CMD_RW_REG_MSK & x))

#endif // __NRF24L01_H__
