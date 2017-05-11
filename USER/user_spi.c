#include "user_spi.h"
#include "nRF24L01.h"
#include "delay.h"

#define SPI_NRF24                    SPI2
#define SPI_NRF24_CLK                RCC_APB1Periph_SPI2
#define SPI_NRF24_GPIO               GPIOB
#define SPI_NRF24_GPIO_CLK           RCC_APB2Periph_GPIOB
#define SPI_NRF24_PIN_NSS			 GPIO_Pin_12
#define SPI_NRF24_PIN_SCK            GPIO_Pin_13
#define SPI_NRF24_PIN_MISO           GPIO_Pin_14
#define SPI_NRF24_PIN_MOSI           GPIO_Pin_15
#define SPI_NRF24_PIN_CE			 GPIO_Pin_0
#define SPI_NRF24_IRQn               SPI2_IRQn

extern GPIO_InitTypeDef GPIO_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;
extern SPI_InitTypeDef SPI_InitStructure;

void SPI_NRF24_Setup(void){
	RCC_APB2PeriphClockCmd(SPI_NRF24_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(SPI_NRF24_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SPI_NRF24_PIN_SCK | SPI_NRF24_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_NRF24_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Software Controller NSS (CSN) Signal of SPI2
	GPIO_InitStructure.GPIO_Pin = SPI_NRF24_PIN_NSS | SPI_NRF24_PIN_CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* PULL Read/Write with SPI2, instead of Interruption
	NVIC_InitStructure.NVIC_IRQChannel = SPI_NRF24_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); */

	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	
	/* Enable SPI2 TXE interrupt */
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
	/* Enable SPI2 RXNE interrupt */
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	SPI_NRF24_CSN_Set(1);
	SPI_NRF24_CE_Set(0);
	delay_msus(101, 0);			// need to delay 101 (100)ms to power on to Power Down Mode
	nRF24_state.mode = POWER_DOWN;
}

void SPI_NRF24_CSN_Set(uint8_t logicValue){
	if (logicValue == 0){
		GPIO_ResetBits(SPI_NRF24_GPIO, SPI_NRF24_PIN_NSS);
	} else {
		GPIO_SetBits(SPI_NRF24_GPIO, SPI_NRF24_PIN_NSS);
		delay_msus(0, 1); // Hold the CSN High for 1us to get ready to next SPI R/W
	}
}

void SPI_NRF24_CE_Set(uint8_t logicValue){
	if (logicValue == 0){
		GPIO_ResetBits(SPI_NRF24_GPIO, SPI_NRF24_PIN_CE);
	} else {
		delay_msus(0, 4);
		GPIO_SetBits(SPI_NRF24_GPIO, SPI_NRF24_PIN_CE);
	}	
}

uint8_t SPI_NRF24_RW(uint8_t data){
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET);
	return SPI_I2S_ReceiveData(SPI2);
}

uint8_t SPI_NRF24_WriteBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size){
	uint8_t status, byte_cnt;
	SPI_NRF24_CSN_Set(0);
	
	status = SPI_NRF24_RW(cmd);
	for (byte_cnt = 0; byte_cnt < size; byte_cnt ++ ){
		SPI_NRF24_RW(pBuf[byte_cnt]);
	}
	
	SPI_NRF24_CSN_Set(1);
	
	return status;
}

uint8_t SPI_NRF24_ReadBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size){
	uint8_t status, byte_cnt;
	SPI_NRF24_CSN_Set(0);
	
	status = SPI_NRF24_RW(cmd);
	for (byte_cnt = 0; byte_cnt < size; byte_cnt ++ ){
		pBuf[byte_cnt] = SPI_NRF24_RW(NRF24_SPI_CMD_NOP);
	}
	
	SPI_NRF24_CSN_Set(1);
	
	return status;
}

uint8_t SPI_NRF24_ReadReg(uint8_t reg){
	uint8_t reg_value = 0;
	SPI_NRF24_CSN_Set(0);
	
	SPI_NRF24_RW( NRF24_R_REG_CMD(reg) );
	reg_value = SPI_NRF24_RW(0);
	SPI_NRF24_CSN_Set(1);
	return reg_value;
}

uint8_t SPI_NRF24_WriteReg(uint8_t reg, uint8_t data){
	uint8_t status = 0;
	SPI_NRF24_CSN_Set(0);
	
	status = SPI_NRF24_RW( NRF24_W_REG_CMD(reg) );
	SPI_NRF24_RW(data);
	SPI_NRF24_CSN_Set(1);
	return status;
}
