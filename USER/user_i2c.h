#ifndef __USER_I2C_H__
#define __USER_I2C_H__

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"

void I2C_Config(I2C_TypeDef* I2Cx);
void I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_Write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_Read_Ack(I2C_TypeDef* I2Cx);
uint8_t I2C_Read_NAck(I2C_TypeDef* I2Cx);
void I2C_Stop(I2C_TypeDef* I2Cx);

// High level read write 
uint8_t I2C_Read_Byte(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra);
void I2C_Write_Byte(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra, uint8_t I2C_s_d);
uint8_t I2C_Burst_Read(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra, uint16_t user_dsize, uint8_t *user_data);
uint8_t I2C_Burst_Write(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra, uint16_t user_dsize, uint8_t *user_data);
uint8_t I2C_Read_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr,
					uint8_t I2C_s_ra,
					uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len);
void I2C_Write_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr,
					uint8_t I2C_s_ra, uint8_t I2C_s_d,
					uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len);

#endif // __USER_I2C_H__
