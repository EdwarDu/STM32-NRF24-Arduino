#ifndef __MPU6050_I2C_H__
#define __MPU6050_I2C_H__

#include "user_i2c.h"

#define MPU6050_ADDRESS									0x68
#define MPU6050_I2C_PORT								I2C2
#define MPU6050_I2C_PERIPH_RCC 							RCC_APB1Periph_I2C2
#define MPU6050_I2C_GPIO								RCC_APB2Periph_GPIOB

#define MPU6050_MY_ID				                    0x68

#define MPU6050_SUCCESS			                        0x00
#define MPU6050_FAILED			                        0x01			/* General Error*/
#define MPU6050_ENABLE									1
#define MPU6050_DISABLE									0
#define MPU6050_INT_L_H									0
#define MPU6050_INT_L_L									1

/*
uint8_t I2C_Read_Byte(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra);
void I2C_Write_Byte((I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra, uint8_t I2C_s_d);
void I2C_Burst_Read(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra, uint8_t *user_data,	uint16_t user_dsize);
void I2C_Burst_Write(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra, const uint8_t *user_data, uint16_t user_dsize);
uint8_t I2C_Read_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr,	uint8_t I2C_s_ra, uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len);
void I2C_Write_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra, uint8_t I2C_s_d, uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len);
*/

#define MPU6050_Read_Reg(MPU6050_reg_addr) 									I2C_Read_Byte(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr)
#define MPU6050_Write_Reg(MPU6050_reg_addr,MPU6050_reg_d) 					I2C_Write_Byte(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr, MPU6050_reg_d)
#define MPU6050_Burst_Read(MPU6050_reg_addr,user_data,size) 				I2C_Burst_Read(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr, user_data, size)
#define MPU6050_Burst_Write(MPU6050_reg_addr,user_data,size) 				I2C_Burst_Write(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr, user_data, size)
#define MPU6050_Read_Bits(MPU6050_reg_addr,bit_pos,bit_len) 				I2C_Read_Bits(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr, bit_pos, bit_len)
#define MPU6050_Write_Bits(MPU6050_reg_addr,MPU6050_reg_d,bit_pos,bit_len) 	I2C_Read_Bits(MPU6050_I2C_PORT, MPU6050_ADDRESS, MPU6050_reg_addr, bit_pos, bit_len)


uint8_t MPU6050_I2C_Init(void);
void MPU6050_Setup(void);
uint16_t MPU6050_Get_Temp(void);


#endif // __MPU6050_I2C_H__
