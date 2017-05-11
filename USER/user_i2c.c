#include "user_i2c.h"

extern GPIO_InitTypeDef  GPIO_InitStructure; 
extern I2C_InitTypeDef I2C_InitStructure;
	
void I2C_Config(I2C_TypeDef* I2Cx){
	if (I2Cx == I2C1){
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;					// PB.6 & 7
	} else if (I2Cx == I2C2) {
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	}
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_DeInit(I2Cx);
	I2C_InitStructure.I2C_ClockSpeed = 400000;  					// 400kHz I2C fast mode
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;						//
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;			// Tlow/Thigh = 2
	I2C_InitStructure.I2C_OwnAddress1 = 0x0000;						// Doesn't Matter ?
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;						// Enable ACK Message
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 0x4000

	I2C_Init(I2Cx, &I2C_InitStructure);
	return ;
}

/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)  != SUCCESS );
	}
	else if(direction == I2C_Direction_Receiver){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)  != SUCCESS);
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_Write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS );
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_Read_Ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS ) ;
	// read data from I2C data register and return data byte
	return I2C_ReceiveData(I2Cx);
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_Read_NAck(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS );
	// read data from I2C data register and return data byte
	return I2C_ReceiveData(I2Cx);
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_Stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t I2C_Read_Byte(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra){
	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Transmitter);
	I2C_Write(I2Cx, I2C_s_ra);
	I2C_Stop(I2Cx);

	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Receiver);
	return I2C_Read_NAck(I2Cx);
}

void I2C_Write_Byte(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, uint8_t I2C_s_ra, uint8_t I2C_s_d){
	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Transmitter);
	I2C_Write(I2Cx, I2C_s_ra);
	I2C_Write(I2Cx, I2C_s_d);
	I2C_Stop(I2Cx);
}

uint8_t I2C_Burst_Read(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra, uint16_t user_dsize,
					uint8_t *user_data){
	uint16_t i = 0;
	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Transmitter);
	I2C_Write(I2Cx, I2C_s_ra);
	I2C_Stop(I2Cx);

	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Receiver);
	for (i = 0; i < user_dsize - 1; i ++){
		user_data[i] = I2C_Read_Ack(I2Cx);
	}
	user_data[i] = I2C_Read_NAck(I2Cx);
	return 0;
}

uint8_t I2C_Burst_Write(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr, 
					uint8_t I2C_s_ra, uint16_t user_dsize,
					uint8_t *user_data){
	uint16_t i = 0;
	I2C_Start(I2Cx, I2C_s_adr << 1, I2C_Direction_Transmitter);
	I2C_Write(I2Cx, I2C_s_ra);
	for (i = 0; i < user_dsize; i ++){
		I2C_Write(I2Cx, user_data[i]);
	}
	I2C_Stop(I2Cx);
	return 0;
}

uint8_t I2C_Read_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr,
					uint8_t I2C_s_ra,
					uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len) {
	uint8_t d = I2C_Read_Byte(I2Cx, I2C_s_adr, I2C_s_ra);
	uint8_t msk = ((1 << I2C_s_bit_len) - 1) << I2C_s_bit_pos;
	return (d & msk) >> I2C_s_bit_pos;
}

void I2C_Write_Bits(I2C_TypeDef* I2Cx, uint8_t I2C_s_adr,
					uint8_t I2C_s_ra, uint8_t I2C_s_d,
					uint8_t I2C_s_bit_pos, uint8_t I2C_s_bit_len) {
	uint8_t d = I2C_Read_Byte(I2Cx, I2C_s_adr, I2C_s_ra);
	uint8_t msk = ((1 << I2C_s_bit_len) - 1) << I2C_s_bit_pos;
	d = ( d & (~msk) ) ^ ( (I2C_s_d << I2C_s_bit_pos) & msk);
	I2C_Write_Byte(I2Cx, I2C_s_adr, I2C_s_ra, d);
}
