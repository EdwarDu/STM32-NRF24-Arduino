#ifndef __MPU6050_REG_DEF_H__
#define __MPU6050_REG_DEF_H__

/*MPU6050 MPU6050 Register Map*/
#define MPU6050_RA_XG_OFFS_TC     						0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC     						0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC     						0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      					0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      					0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      					0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        					0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     					0x07
#define MPU6050_RA_YA_OFFS_H        					0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     					0x09
#define MPU6050_RA_ZA_OFFS_H        					0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     					0x0B

#define MPU6050_RA_ACCEL_SHIFT							0x0d // 6 after this is the Accel Prod(uct?) Shift

#define MPU6050_RA_XG_OFFS_USRH     					0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     					0x14
#define MPU6050_RA_YG_OFFS_USRH     					0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     					0x16
#define MPU6050_RA_ZG_OFFS_USRH     					0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     					0x18

#define MPU6050_RA_AUX_VDDIO							0x01
#define MPU6050_RA_AUX_VDDIO_BIT						7  // 0b10000000	/* 1 => VDD, 0 => VLOGIC */

#define MPU6050_RA_SMPRT_DIV							0x19		/* Sample Rate Divider, 8bit UNSIGNED  SampleRate = Gyroscope Output Rate / (1 + SMPRT_DIV) */

#define MPU6050_RA_CONFIG								0x1a		/* Configuration */
#define MPU6050_RA_CONFIG_EXT_SYNC_SET_MSK				0x38 // 0b00111000	/* External Frame Synchronization (FSYNC) pin Sampling --NOT USED in MPU6050-- */
#define MPU6050_RA_CONFIG_EXT_SYNC_SET_BIT				3
#define MPU6050_RA_CONFIG_EXT_SYNC_SET_LEN				3
/* MPU6050_RA_CONFIG_MSK_EXT_SYNC_SET 
0 		Input Disabled
1		TEMP_OUT_L[0]
2		GYRO_XOUT_L[0]
3 		GYRO_YOUT_L[0]
4		GYRO_ZOUT_L[0]
5		ACCEL_XOUT_L[0]
6		ACCEL_YOUT_L[0]
7		ACCEL_ZOUT_L[0] */
#define MPU6050_RA_CONFIG_DLPF_CFG_MSK					0x07 // 0b00000111	/* Digital Low Pass Filter (DLPF) for both Gyro and Accl */
#define MPU6050_RA_CONFIG_DLPF_CFG_BIT					0
#define MPU6050_RA_CONFIG_DLPF_CFG_LEN					3
/* MPU6050_RA_CONFIG_MSK_DLPF_CFG
		 		Accel (Fs = 1KHz) 							Gyro
DLPF_CFG 	Bandwidth(Hz)	Delay(ms)		Bandwidth(Hz)	Delay(ms)	Fs(kHz)
0 			260				0 				256 			0.98 		8
1   		184				2.0 			188 			1.9 		1
2 			94 				3.0 			98 				2.8 		1
3 			44 				4.9 			42 				4.8 		1
4 			21				8.5 			20 				8.3 		1
5 			10 				13.8 			10 				13.4 		1
6			5 				19.0 			5 				18.6 		1
7 					RESERVED					RESERVED 				8 */

#define MPU6050_RA_GYRO_CONFIG							0x1b
#define MPU6050_RA_GYRO_CONFIG_XG_ST_BIT				7
#define MPU6050_RA_GYRO_CONFIG_YG_ST_BIT				6
#define MPU6050_RA_GYRO_CONFIG_ZG_ST_BIT				5
#define MPU6050_RA_GYRO_CONFIG_FS_SEL_BIT				3
#define MPU6050_RA_GYRO_CONFIG_FS_SEL_LEN				2
#define MPU6050_RA_GYRO_CONFIG_FS_SEL_MSK				0x18 // 0b00011000	/* Full Scale Range */
/* MPU6050_RA_GYRO_CONFIG_MSK_FS_SEL
FS_SEL 		Full Scale Range
0 			+/- 250 degree / s
1 			+/- 500
2 			+/- 1000
3 			+/- 2000 */

#define MPU6050_RA_ACCEL_CONFIG							0x1c
#define MPU6050_RA_ACCEL_CONFIG_XA_ST_BIT				7
#define MPU6050_RA_ACCEL_CONFIG_YA_ST_BIT				6
#define MPU6050_RA_ACCEL_CONFIG_ZA_ST_BIT 				5
#define MPU6050_RA_ACCEL_CONFIG_AFS_SEL_BIT				3
#define MPU6050_RA_ACCEL_CONFIG_AFS_SEL_LEN				2
/* MPU6050_RA_ACCEL_CONFIG_MSK_AFS_SEL
AFS_SEL 		Full Scale Range
0 				+/- 2g
1 				+/- 4g
2 				+/- 8g
3 				+/- 16g */
#define MPU6050_RA_ACCEL_CONFIG_ACCEL_HPF_BIT			0
#define MPU6050_RA_ACCEL_CONFIG_ACCEL_HPF_LEN			3
/* MPU6050_RA_ACCEL_CONFIG_MSK_ACCEL_HPF
Reset: 	The filter output settles to zero within one sample. This effectively disables the high
		pass filter. This mode may be toggled to quickly settle the filter.
On:		The high pass filter will pass signals above the cut off frequency
Hold: 	When triggered, the filter holds the present sample. The filter output will be the
		difference between the input sample and the held sample.
ACCEL_HPF 	Filter Mode 	Cut-off Frequency
0 			Reset 			None
1 			On 				5Hz
2 			On 				2.5Hz
3 			On 				1.25Hz
4 			On 				0.63Hz
7 			Hold 			None */

#define MPU6050_RA_FF_THR								0x1d  /* Free Fall Acceleration Threshold [TODO] mg per LSB incr*/
#define MPU6050_RA_FF_DUR 								0x1e  /* Free Fall Duration (counter @ 1kHz => unit 1ms)*/
#define MPU6050_RA_MOT_THR								0x1f  /* Motion Detection Threshold */
#define MPU6050_RA_MOT_DUR								0x20  /* Motion Detection Duration (counter @ 1kHz => unit 1ms)*/
#define MPU6050_RA_ZRMOT_THR							0x21  /* Zero Motion Detection Threshold */
#define MPU6050_RA_ZRMOT_DUR							0x22  /* Zero Motion Detection Duration (counter @ 16Hz => uint 64ms)*/

#define MPU6050_RA_FIFO_EN								0x23  /* FIFO Enable */
#define MPU6050_RA_FIFO_EN_TEMP_BIT						7  // 0b10000000 /*TEMP_OUT_H/_L => FIFO buffer */
#define MPU6050_RA_FIFO_EN_XG_BIT						6  // 0b01000000 /*GYRO_XOUT_H/_L => FIFO buffer */
#define MPU6050_RA_FIFO_EN_YG_BIT						5  // 0b00100000 /*GYRO_YOUT_H/_L => FIFO buffer */
#define MPU6050_RA_FIFO_EN_ZG_BIT						4  // 0b00010000 /*GYRO_ZOUT_H/_L => FIFO buffer */
#define MPU6050_RA_FIFO_EN_ACCEL_BIT					3  // 0b00001000 /*ACCEL_X&Y&ZOUT_H/_L => FIFO buffer */
#define MPU6050_RA_FIFO_EN_SLV2_BIT						2  // 0b00000100 /*External I2C Slave2 => FIFO buffer */
#define MPU6050_RA_FIFO_EN_SLV1_BIT						1  // 0b00000010 /*External I2C Slave1 => FIFO buffer */
#define MPU6050_RA_FIFO_EN_SLV0_BIT						0  // 0b00000001 /*External I2C Slave0 => FIFO buffer */
/* For SLV2-0 Refer to Register 73-96 */

#define MPU6050_RA_I2C_MST_CTRL							0x24
#define MPU6050_RA_I2C_MST_CTRL_MULT_MST_BIT			7  // 0b10000000 /*Multi-Master Enable */
#define MPU6050_RA_I2C_MST_CTRL_WAIT4ES_BIT				6  // 0b01000000 /*Data ready interrupt will delayed till external sensor data ready*/
#define MPU6050_RA_I2C_MST_CTRL_SLV3_BIT				5  // 0b00100000 /*External I2C Slave3 => FIFO buffer */
#define MPU6050_RA_I2C_MST_CTRL_I2C_MST_P_NSR_BIT		4  // 0b00010000
/*If the bit equals 0, there will be a restart between reads. If the bit equals 1, there will be a
stop followed by a start of the following read. When a write transaction follows a read transaction, the
stop followed by a start of the successive write will be always used. */
#define MPU6050_RA_I2C_MST_CTRL_I2C_MST_CLK_BIT			0
#define MPU6050_RA_I2C_MST_CTRL_I2C_MST_CLK_LEN			4
#define MPU6050_RA_I2C_MST_CTRL_I2C_MST_CLK_MSK			0x0F  // 0b00001111
/* MPU6050_RA_I2C_MST_CTRL_MSK_I2C_MST_CLK
I2C_MST_CLK 	I2C Master Clock Speed	8MHz Clock Divider
0 				348 kHz 				23
1 				333						24 		
2 				320						25
3				308						26
4				296						27
5				286						28
6				276						29
7				267						30
8				258						31
9				500						16
10				471						17
11				444						18
12				421						19
13				400						20
14				381						21
15				364						22 */


#define MPU6050_RA_I2C_SLV0_CFG0						0x25
#define MPU6050_RA_I2C_SLV0_CFG0_RW_BIT					7  // 0b10000000 (1 => Read Op; 0 => Write Op)
#define MPU6050_RA_I2C_SLV0_CFG0_ADDR_BIT				0
#define MPU6050_RA_I2C_SLV0_CFG0_ADDR_LEN				7
#define MPU6050_RA_I2C_SLV0_CFG0_ADDR_MSK				0x7f

#define MPU6050_RA_I2C_SLV0_CFG1						0x26  // I2C_SLV0_REG address
#define MPU6050_RA_I2C_SLV0_CFG2						0x27
#define MPU6050_RA_I2C_SLV0_CFG2_EN_BIT					7  // 0b10000000
#define MPU6050_RA_I2C_SLV0_CFG2_BYTE_SW_BIT			6  // 0b01000000
#define MPU6050_RA_I2C_SLV0_CFG2_REG_DIS_BIT			5  // 0b00100000
#define MPU6050_RA_I2C_SLV0_CFG2_GRP_BIT				4  // 0b00010000
#define MPU6050_RA_I2C_SLV0_CFG2_LEN_BIT				0
#define MPU6050_RA_I2C_SLV0_CFG2_LEN_LEN				0
#define MPU6050_RA_I2C_SLV0_CFG2_LEN_MSK				0x0F  // 0b00001111

#define MPU6050_RA_I2C_SLV1_CFG0						0x28
#define MPU6050_RA_I2C_SLV1_CFG0_RW_BIT					7  // 0b10000000 (1 => Read Op; 0 => Write Op)
#define MPU6050_RA_I2C_SLV1_CFG0_ADDR_BIT				0
#define MPU6050_RA_I2C_SLV1_CFG0_ADDR_LEN				7
#define MPU6050_RA_I2C_SLV1_CFG0_ADDR_MSK				0x7F

#define MPU6050_RA_I2C_SLV1_CFG1						0x29  // I2C_SLV1_REG address
#define MPU6050_RA_I2C_SLV1_CFG2						0x2a
#define MPU6050_RA_I2C_SLV1_CFG2_EN_BIT					7  // 0b10000000
#define MPU6050_RA_I2C_SLV1_CFG2_BYTE_SW_BIT			6  // 0b01000000
#define MPU6050_RA_I2C_SLV1_CFG2_REG_DIS_BIT			5  // 0b00100000
#define MPU6050_RA_I2C_SLV1_CFG2_GRP_BIT				4  // 0b00010000
#define MPU6050_RA_I2C_SLV1_CFG2_LEN_BIT				0
#define MPU6050_RA_I2C_SLV1_CFG2_LEN_LEN				4
#define MPU6050_RA_I2C_SLV1_CFG2_LEN_MSK				0x0F  // 0b00001111

#define MPU6050_RA_I2C_SLV2_CFG0						0x2b
#define MPU6050_RA_I2C_SLV2_CFG0_RW_BIT					7  // 0b10000000 (1 => Read Op; 0 => Write Op)
#define MPU6050_RA_I2C_SLV2_CFG0_ADDR_BIT				0
#define MPU6050_RA_I2C_SLV2_CFG0_ADDR_LEN				7
#define MPU6050_RA_I2C_SLV2_CFG0_ADDR_MSK				0x7f

#define MPU6050_RA_I2C_SLV2_CFG1						0x2c  // I2C_SLV2_REG address
#define MPU6050_RA_I2C_SLV2_CFG2						0x2d
#define MPU6050_RA_I2C_SLV2_CFG2_EN_BIT					7  // 0b10000000
#define MPU6050_RA_I2C_SLV2_CFG2_BYTE_SW_BIT			6  // 0b01000000
#define MPU6050_RA_I2C_SLV2_CFG2_REG_DIS_BIT			5  // 0b00100000
#define MPU6050_RA_I2C_SLV2_CFG2_GRP_BIT				4  // 0b00010000
#define MPU6050_RA_I2C_SLV2_CFG2_LEN_BIT				0
#define MPU6050_RA_I2C_SLV2_CFG2_LEN_LEN				0
#define MPU6050_RA_I2C_SLV2_CFG2_LEN_MSK				0x0F  // 0b00001111

#define MPU6050_RA_I2C_SLV3_CFG0						0x2e
#define MPU6050_RA_I2C_SLV3_CFG0_RW_BIT					7  // 0b10000000 (1 => Read Op; 0 => Write Op)
#define MPU6050_RA_I2C_SLV3_CFG0_ADDR_BIT				0
#define MPU6050_RA_I2C_SLV3_CFG0_ADDR_LEN				7
#define MPU6050_RA_I2C_SLV3_CFG0_ADDR_MSK				0x7f

#define MPU6050_RA_I2C_SLV3_CFG1						0x2f  // I2C_SLV3_REG address
#define MPU6050_RA_I2C_SLV3_CFG2						0x30
#define MPU6050_RA_I2C_SLV3_CFG2_EN_BIT					7  // 0b10000000
#define MPU6050_RA_I2C_SLV3_CFG2_BYTE_SW_BIT			6  // 0b01000000
#define MPU6050_RA_I2C_SLV3_CFG2_REG_DIS_BIT			5  // 0b00100000
#define MPU6050_RA_I2C_SLV3_CFG2_GRP_BIT				4  // 0b00010000
#define MPU6050_RA_I2C_SLV3_CFG2_LEN_BIT				0
#define MPU6050_RA_I2C_SLV3_CFG2_LEN_LEN				0
#define MPU6050_RA_I2C_SLV3_CFG2_LEN_MSK				0x0F  // 0b00001111

#define MPU6050_RA_I2C_SLV4_CFG0						0x31
#define MPU6050_RA_I2C_SLV4_CFG0_RW_BIT					7  // 0b10000000 (1 => Read Op; 0 => Write Op)
#define MPU6050_RA_I2C_SLV4_CFG0_ADDR_BIT				0
#define MPU6050_RA_I2C_SLV4_CFG0_ADDR_LEN				7
#define MPU6050_RA_I2C_SLV4_CFG0_ADDR_MSK				0x7F

#define MPU6050_RA_I2C_SLV4_CFG1						0x32  // I2C_SLV4_REG address
#define MPU6050_RA_I2C_SLV4_CFG2						0x33  // I2C_SLV4_DO
#define MPU6050_RA_I2C_SLV4_CFG3						0x34
#define MPU6050_RA_I2C_SLV4_CFG3_EN_BIT					7
#define MPU6050_RA_I2C_SLV4_CFG3_INT_EN_BIT				6
#define MPU6050_RA_I2C_SLV4_CFG3_REG_DIS_BIT			5
#define MPU6050_RA_I2C_SLV4_CFG3_MST_DLY_BIT			0
#define MPU6050_RA_I2C_SLV4_CFG3_MST_DLY_LEN			5
#define MPU6050_RA_I2C_SLV4_CFG3_MST_DLY_MSK			0x1F  // 0b00011111
#define MPU6050_RA_I2C_SLV4_CFG4						0x35  // I2C_SLV4_DI

#define MPU6050_RA_I2C_MST_STATUS						0x36  // I2C Master Status (RO)
#define MPU6050_RA_I2C_MST_STATUS_PASS_THRO_BIT			7
#define MPU6050_RA_I2C_MST_STATUS_SLV4_DONE_BIT			6
#define MPU6050_RA_I2C_MST_STATUS_LOST_ARB_BIT			5
#define MPU6050_RA_I2C_MST_STATUS_SLV4_NACK_BIT			4
#define MPU6050_RA_I2C_MST_STATUS_SLV3_NACK_BIT			3
#define MPU6050_RA_I2C_MST_STATUS_SLV2_NACK_BIT			2
#define MPU6050_RA_I2C_MST_STATUS_SLV1_NACK_BIT			1
#define MPU6050_RA_I2C_MST_STATUS_SLV0_NACK_BIT			0

#define MPU6050_RA_INT_PIN_CFG							0x37  // INT Pin/Bypass Enable Configuration
#define MPU6050_RA_INT_PIN_CFG_INT_LVL_BIT				7
#define MPU6050_RA_INT_PIN_CFG_INT_OPEN_BIT				6
#define MPU6050_RA_INT_PIN_CFG_LATCH_INT_EN_BIT			5
#define MPU6050_RA_INT_PIN_CFG_INT_RD_CLEAR_BIT			4
#define MPU6050_RA_INT_PIN_CFG_FSYNC_INT_LVL_BIT		3
#define MPU6050_RA_INT_PIN_CFG_FSYNC_INT_EN_BIT			2
#define MPU6050_RA_INT_PIN_CFG_I2C_BYPASS_EN_BIT		1
#define MPU6050_RA_INT_PIN_CFG_CLKOUT_EN_BIT			0

#define MPU6050_RA_INT_EN 								0x38
#define MPU6050_RA_INT_EN_FF_BIT 						7  /*Fall Free*/
#define MPU6050_RA_INT_EN_MOT_BIT 	 					6  /*Motion Detect*/
#define MPU6050_RA_INT_EN_ZMOT_BIT 	 					5  /*Zero Motion Detect*/
#define MPU6050_RA_INT_EN_FIFO_OFLOW_BIT 				4  /*FIFO Overflow*/
#define MPU6050_RA_INT_EN_I2C_MST_INT_BIT 				3  /* */
#define MPU6050_RA_INT_EN_D_RDY_BIT  					0  /*Data Ready*/

#define MPU6050_RA_DMP_INT_STATUS   					0x39

#define MPU6050_RA_INT_STATUS							0x3a
#define MPU6050_RA_INT_STATUS_FF_BIT					7  /*Fall Free*/
#define MPU6050_RA_INT_STATUS_MOT_BIT 					6  /*Motion Detect*/
#define MPU6050_RA_INT_STATUS_ZMOT_BIT 					5  /*Zero Motion Detect*/
#define MPU6050_RA_INT_STATUS_FIFO_OFLOW_BIT			4  /*FIFO Overflow*/
#define MPU6050_RA_INT_STATUS_I2C_MST_INT_BIT 			3  /* */
#define MPU6050_RA_INT_STATUS_D_RDY_BIT					0  /*Data Ready*/

#define MPU6050_RA_ACCEL_XOUT_H 						0x3b
#define MPU6050_RA_ACCEL_XOUT_L							0x3c
#define MPU6050_RA_ACCEL_YOUT_H 						0x3d
#define MPU6050_RA_ACCEL_YOUT_L 						0x3e
#define MPU6050_RA_ACCEL_ZOUT_H 						0x3f
#define MPU6050_RA_ACCEL_ZOUT_L							0x40
/*
AFS_SEL 	Full Scale Range 	LSB Sensitivity
0 			+/- 2g				16384 LSB/mg
1 			+/- 4g				8192 LSB/mg
2 			+/- 8g				4096 LSB/mg
3 			+/- 16g				2046 LSB/mg */

#define MPU6050_RA_TEMP_OUT_H							0x41
#define MPU6050_RA_TEMP_OUT_L 							0x42

#define MPU6050_RA_GYRO_XOUT_H							0x43
#define MPU6050_RA_GYRO_XOUT_L 							0x44
#define MPU6050_RA_GYRO_YOUT_H							0x45
#define MPU6050_RA_GYRO_YOUT_L 							0x46
#define MPU6050_RA_GYRO_ZOUT_H							0x47
#define MPU6050_RA_GYRO_ZOUT_L							0x48
/*
FS_SEL 		Full Scale Range 	LSB Sensitivity
0 			+/- 250 degree/s 	131 LSB/degree/s
0 			+/- 500 degree/s 	65.5 LSB/degree/s
0 			+/- 1000 degree/s 	32.8 LSB/degree/s
0 			+/- 2000 degree/s 	16.4 LSB/degree/s */

#define MPU6050_RA_EXT_SENS_DATA_00 					0x49
#define MPU6050_RA_EXT_SENS_DATA_01 					0x4a
#define MPU6050_RA_EXT_SENS_DATA_02 					0x4b
#define MPU6050_RA_EXT_SENS_DATA_03 					0x4c
#define MPU6050_RA_EXT_SENS_DATA_04 					0x4d
#define MPU6050_RA_EXT_SENS_DATA_05 					0x4e
#define MPU6050_RA_EXT_SENS_DATA_06 					0x4f
#define MPU6050_RA_EXT_SENS_DATA_07 					0x50
#define MPU6050_RA_EXT_SENS_DATA_08 					0x51
#define MPU6050_RA_EXT_SENS_DATA_09 					0x52
#define MPU6050_RA_EXT_SENS_DATA_10 					0x53
#define MPU6050_RA_EXT_SENS_DATA_11 					0x54
#define MPU6050_RA_EXT_SENS_DATA_12 					0x55
#define MPU6050_RA_EXT_SENS_DATA_13 					0x56
#define MPU6050_RA_EXT_SENS_DATA_14 					0x57
#define MPU6050_RA_EXT_SENS_DATA_15 					0x58
#define MPU6050_RA_EXT_SENS_DATA_16 					0x59
#define MPU6050_RA_EXT_SENS_DATA_17 					0x5a
#define MPU6050_RA_EXT_SENS_DATA_18 					0x5b
#define MPU6050_RA_EXT_SENS_DATA_19 					0x5c
#define MPU6050_RA_EXT_SENS_DATA_20 					0x5d
#define MPU6050_RA_EXT_SENS_DATA_21						0x5e
#define MPU6050_RA_EXT_SENS_DATA_22 					0x5f
#define MPU6050_RA_EXT_SENS_DATA_23 					0x60

#define MPU6050_RA_MOT_DET_STATUS 						0x61
#define MPU6050_RA_MOT_DET_STATUS_XNEG_BIT 				7
#define MPU6050_RA_MOT_DET_STATUS_XPOS_BIT 				6
#define MPU6050_RA_MOT_DET_STATUS_YNEG_BIT 				5
#define MPU6050_RA_MOT_DET_STATUS_YPOS_BIT 				4
#define MPU6050_RA_MOT_DET_STATUS_ZNEG_BIT 				3
#define MPU6050_RA_MOT_DET_STATUS_ZPOS_BIT 				2
#define MPU6050_RA_MOT_DET_STATUS_ZRMOT_BIT 			0

#define MPU6050_RA_I2C_SLV0_DO							0x63
#define MPU6050_RA_I2C_SLV1_DO							0x64
#define MPU6050_RA_I2C_SLV2_DO							0x65
#define MPU6050_RA_I2C_SLV3_DO							0x66

#define MPU6050_RA_I2C_MST_DLY_CTRL						0x67
#define MPU6050_RA_I2C_MST_DLY_CTRL_DELAY_ES_SHADOW_BIT	7
#define MPU6050_RA_I2C_MST_DLY_CTRL_SLV4_DLY_EN_BIT		4
#define MPU6050_RA_I2C_MST_DLY_CTRL_SLV3_DLY_EN_BIT		3
#define MPU6050_RA_I2C_MST_DLY_CTRL_SLV2_DLY_EN_BIT		2
#define MPU6050_RA_I2C_MST_DLY_CTRL_SLV1_DLY_EN_BIT		1
#define MPU6050_RA_I2C_MST_DLY_CTRL_SLV0_DLY_EN_BIT		0

#define MPU6050_RA_SIG_PATH_RST							0x68
#define MPU6050_RA_SIG_PATH_RST_GYRO_BIT				2
#define MPU6050_RA_SIG_PATH_RST_ACCEL_BIT				1
#define MPU6050_RA_SIG_PATH_RST_TEMP_BIT				0

#define MPU6050_RA_MOT_DET_CTRL							0x69
#define MPU6050_RA_MOT_DET_CTRL_ACCEL_ON_DLY_BIT		4
#define MPU6050_RA_MOT_DET_CTRL_ACCEL_ON_DLY_LEN		2
#define MPU6050_RA_MOT_DET_CTRL_ACCEL_ON_DLY_MSK		0x30
#define MPU6050_RA_MOT_DET_CTRL_FF_COUNT_BIT			2
#define MPU6050_RA_MOT_DET_CTRL_FF_COUNT_LEN			2
#define MPU6050_RA_MOT_DET_CTRL_FF_COUNT_MSK			0x0C
#define MPU6050_RA_MOT_DET_CTRL_MOT_COUNT_BIT			0
#define MPU6050_RA_MOT_DET_CTRL_MOT_COUNT_LEN			2			
#define MPU6050_RA_MOT_DET_CTRL_MOT_COUNT_MSK			0x03

#define MPU6050_RA_USER_CTRL							0x6a
#define MPU6050_RA_USER_CTRL_FIFO_EN_BIT				6
#define MPU6050_RA_USER_CTRL_I2C_MST_EN_BIT				5
#define MPU6050_RA_USER_CTRL_I2C_IF_DIS_BIT				4
#define MPU6050_RA_USER_CTRL_FIFO_RST_BIT				2
#define MPU6050_RA_USER_CTRL_I2C_MST_RST_BIT			1
#define MPU6050_RA_USER_CTRL_SIG_COND_RST_BIT			0

#define MPU6050_RA_PWR_MGMT1							0x6b
#define MPU6050_RA_PWR_MGMT1_DEV_RST_BIT				7
#define MPU6050_RA_PWR_MGMT1_SLEEP_BIT					6
#define MPU6050_RA_PWR_MGMT1_CYCLE_BIT					5
#define MPU6050_RA_PWR_MGMT1_TEMP_DIS_BIT				3
#define MPU6050_RA_PWR_MGMT1_CLKSEL_BIT					0
#define MPU6050_RA_PWR_MGMT1_CLKSEL_LEN					3
#define MPU6050_RA_PWR_MGMT1_CLKSEL_MSK					0x07
/* MPU6050_RA_PWR_MGMT1_MSK_CLKSEL
CLKSEL 		Clock Source
0 			Internal 8MHz Oscillator
1 			PLLL with X axis gyroscope reference
2 			PLLL with Y axis gyroscope reference
3 			PLLL with Z axis gyroscope reference
4 			PLLL with external 32.768 kHz reference
5 			PLLL with external 19.2 MHz reference
6 			Reserved
7 			Stops the clock and keeps the timing generator in reset */

#define MPU6050_RA_PWR_MGMT2							0x6c
#define MPU6050_RA_PWR_MGMT2_LP_WAKE_CTRL_BIT			6
#define MPU6050_RA_PWR_MGMT2_LP_WAKE_CTRL_LEN			2
#define MPU6050_RA_PWR_MGMT2_LP_WAKE_CTRL_MSK			0xc0
/* MPU6050_RA_PWR_MGMT2_MSK_LP_WAKE_CTRL
LP_WAKE_CTRL 	Wake-up Frequency
0 				1.25 Hz
1 				2.5 Hz
2 				5 Hz
3 				10 Hz */
#define MPU6050_RA_PWR_MGMT2_STBY_XA_BIT				5
#define MPU6050_RA_PWR_MGMT2_STBY_YA_BIT				4
#define MPU6050_RA_PWR_MGMT2_STBY_ZA_BIT				3
#define MPU6050_RA_PWR_MGMT2_STBY_XG_BIT				2
#define MPU6050_RA_PWR_MGMT2_STBY_YG_BIT				1
#define MPU6050_RA_PWR_MGMT2_STBY_ZG_BIT				0

#define MPU6050_RA_BANK_SEL         					0x6D
#define MPU6050_RA_MEM_START_ADDR   					0x6E
#define MPU6050_RA_MEM_R_W          					0x6F
#define MPU6050_RA_DMP_CFG_1        					0x70
#define MPU6050_RA_DMP_CFG_2        					0x71

#define MPU6050_RA_FIFO_COUNT_H							0x72
#define MPU6050_RA_FIFO_COUNT_L							0x73

#define MPU6050_RA_FIFO_DATA							0x74


#define MPU6050_RA_WHOAMI								0x75
#define MPU6050_RA_WHOAMI_MSK							0x7E

#define MPU6050_WHOAMI_ID								0x68

#endif // __MPU6050_REG_DEF_H__