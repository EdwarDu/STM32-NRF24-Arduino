#ifndef __USER_SPI_H__
#define __USER_SPI_H__

#include "stm32f10x.h"
#include "stm32f10x_spi.h"

void SPI_NRF24_Setup(void);

void SPI_NRF24_CSN_Set(uint8_t logicValue);
void SPI_NRF24_CE_Set(uint8_t logicValue);

uint8_t SPI_NRF24_RW(uint8_t data);
uint8_t SPI_NRF24_WriteBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size);
uint8_t SPI_NRF24_ReadBuf(uint8_t cmd, uint8_t *pBuf, uint8_t size);
uint8_t SPI_NRF24_ReadReg(uint8_t reg);
uint8_t SPI_NRF24_WriteReg(uint8_t reg, uint8_t data);
#endif // __USER_SPI_H__
