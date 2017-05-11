/**
  ******************************************************************************
  * @file    USART/USER/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "user_usart.h"
#include "user_i2c.h"
#include "user_exti.h"
#include "user_spi.h"
#include "user_pwm.h"
#include "delay.h"
#include "nRF24L01.h"
#include "stm32f10x.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "hc_sr04.h"
#include "bluetooth.h"
#include "voltage_adc.h"


/*******************Global Variables to Save Space**********************/
GPIO_InitTypeDef			GPIO_InitStructure;
NVIC_InitTypeDef			NVIC_InitStructure;
EXTI_InitTypeDef			EXTI_InitStructure;
SPI_InitTypeDef				SPI_InitStructure;
I2C_InitTypeDef				I2C_InitStructure;
TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
TIM_OCInitTypeDef			TIM_OCInitStructure;
USART_InitTypeDef 			USART_InitStructure;
ADC_InitTypeDef             ADC_InitStructure;
DMA_InitTypeDef 			DMA_InitStructure;

#define USE_NRF24_CONTROL

#define DEFAULT_MPU_HZ  (100)
static signed char gyro_orientation[9] = {1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row){
    unsigned short b;

    if (row[0] > 0)         b = 0;
    else if (row[0] < 0)    b = 4;
    else if (row[1] > 0)    b = 1;
    else if (row[1] < 0)    b = 5;
    else if (row[2] > 0)    b = 2;
    else if (row[2] < 0)    b = 6;
    else                    b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx){
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}


static int run_self_test(void){
	int result;
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
 
/** This part is necessary if you want the position when runing this self-test becomes
	home position, which should NOT be here for flight control
	if ((result & 0x03) == 0x03) {
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
	} */

	return result;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define q30  1073741824.0f
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float yaw, pitch, roll;
int8_t mpu_data_valid = 0;
int8_t hmc_data_valid = 0;

short compass_xyz[3];
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];

uint8_t mpu_status = 0x00;
uint8_t nrf24_status = 0x00;
int16_t adc1_value = -1;

void mpu_exti_handler_cb(void){
	hmc_data_valid = mpu_get_compass_reg(compass_xyz, &sensor_timestamp); 
	more = 1;
	while (more != 0){
		mpu_data_valid = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
				&more);
	
		if ((more == 0) && (sensors & INV_WXYZ_QUAT)) {
			q0=quat[0] / q30;
			q1=quat[1] / q30;
			q2=quat[2] / q30;
			q3=quat[3] / q30;
#define DEGREE_OVER_PI 57.32484f
			pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* DEGREE_OVER_PI; // pitch
			roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* DEGREE_OVER_PI; // roll
			yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * DEGREE_OVER_PI;
		}
		
		/*
		if (sensors & INV_XYZ_GYRO){
		}
		
		if (sensors & INV_XYZ_ACCEL){
		}*/
	}
}


#define MPU_INIT_GOOD				0x80
#define MPU_DMP_GOOD				0x40

#define MPU_ST_GYRO_GOOD			0x02
#define MPU_ST_ACCL_GOOD			0x01
#define MPU_ST_COMP_GOOD			0x04

#define NRF24_CHECK_GOOD			0x80

#define BATT_V_CRITICAL             0x940 // 0x940 * 5 * 3.3 /  4096 = 9.539V

float desiredYaw = 0, desiredRoll = 0, desiredPitch = 0;
short desiredAX = 0, desiredAY = 0, desiredAZ = 0;
uint16_t desireDist = 0;
uint8_t base_th[4] = {0x10, 0x10, 0x10, 0x10};

uint8_t pilot_cmd_act = 0x00;
uint8_t pilot_cmd_desire = 0x00;

void pid_ypr(void){
    // pid process to control the motors according to desired YPR
}

void pid_accelxyz(void){
    // pid process to control the motors according to desired Accel, this may not be good
}

void pid_act(void){

}

void base_test(void){
    TIM5_PWM_Set_Throttle(1, base_th[0]);
    TIM5_PWM_Set_Throttle(2, base_th[1]);
    TIM5_PWM_Set_Throttle(3, base_th[2]);
    TIM5_PWM_Set_Throttle(4, base_th[3]);
    //iored_printf("%d %d %d %d\r\n", base_th[0], base_th[1], base_th[2], base_th[3]);
    return ;
}

#if defined(SE_BT_CONTROL)
    // Different types of BT packets
    // Payload Size == 0 Keep alive packet ACK: BA ID 02 BT_ACK_DONE FF AE
    // Payload Size == 1
    // Type: Data Acquisition  : DATA_ACQ_CMD DATA_FILTER1 DATA_FILTER2 EE CRC
    #define DATA_ACQ_CMD                0x81
    #define DF1_GYRO_XYZ                0x01
    #define DF1_ACCL_XYZ                0x02
    #define DF1_COMP_XYZ                0x04
    #define DF1_MPU_YPR                 0x08
    #define DF1_DOF_ALL                 0x0F

    #define DF1_BATT_VOLTAGE            0x10
    #define DF1_MOTOR_1234              0x20
    #define DF1_DIST					0x40

    #define DF2_MPU_STATUS				0x01
    #define DF2_NRF24_STATUS			0x02
    #define DF2_SYSTICK					0x04

    // Type： Raw Motor Control : MOTOR_CTRL_CMD M1_THROTTLE M2_THROTTLE M3_THROTTLE M4_THROTTLE EE CRC
    #define MOTOR_CTRL_CMD              0x82
    // M*_THROTTLE [0-255]

    #define MODE_CTRL_CMD				0x83
    #define MODE_RAW_MOTOR_CTRL			0xE1

    // Type: Simple Action Control : ACTION_CTRL_CMD ACTION
    // #define ACTION_CTRL_CMD             0x84
    // make the quadcopter hover (at a certain altitude)
    // ACTION_Q_HOEVER [Distance]
    // #define ACTION_Q_HOVER				0x00

    // Type: User set desired Yaw, Pitch, Roll values (in degree*100)
    #define USER_CTRL_CMD               0x84

    extern bt_packet_s cur_bt_packet;
    extern unsigned long SysTick_ts_us;

    uint8_t bt_packet_handler(uint8_t *ack_payload){
        // [TODO] Decode the packet and if required return data in ack_payload
        uint8_t ack_datasize = 0;
    	
    	/*int i = 0;
    	for (i = 0; i < cur_bt_packet.payload_size; i ++){
    		printf("%X ", cur_bt_packet.payload[i]);
    	}
    	printf("\r\r\n");*/
    		
        switch(cur_bt_packet.payload[0]){
            case DATA_ACQ_CMD:{
                uint8_t df1 = cur_bt_packet.payload[1];
                bt_SendACKWithPayload_Init();
                if (df1 & DF1_GYRO_XYZ){
                    ack_payload[++ack_datasize] = (gyro[0] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = gyro[0] & 0xFF;
                    ack_payload[++ack_datasize] = (gyro[1] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = gyro[1] & 0xFF;
                    ack_payload[++ack_datasize] = (gyro[2] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = gyro[2] & 0xFF;
                }
                
                if (df1 & DF1_ACCL_XYZ){
                    ack_payload[++ack_datasize] = (accel[0] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = accel[0] & 0xFF;
                    ack_payload[++ack_datasize] = (accel[1] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = accel[1] & 0xFF;
                    ack_payload[++ack_datasize] = (accel[2] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = accel[2] & 0xFF;
                }
                
                if (df1 & DF1_COMP_XYZ){
                    ack_payload[++ack_datasize] = (compass_xyz[0] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = compass_xyz[0] & 0xFF;
                    ack_payload[++ack_datasize] = (compass_xyz[1] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = compass_xyz[1] & 0xFF;
                    ack_payload[++ack_datasize] = (compass_xyz[2] & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = compass_xyz[2] & 0xFF;
                }
                
                if (df1 & DF1_MPU_YPR){
                    short yaw_16 = yaw*100 + 0.5;
                    short pitch_16 = pitch*100 + 0.5;
                    short roll_16 = roll*100 + 0.5;
                    ack_payload[++ack_datasize] = (yaw_16 & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = yaw_16 & 0xFF;
                    ack_payload[++ack_datasize] = (pitch_16 & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = pitch_16 & 0xFF;
                    ack_payload[++ack_datasize] = (roll_16 & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = roll_16 & 0xFF;
                }
                
                if (df1 & DF1_DOF_ALL){
                    ack_payload[++ack_datasize] = (sensor_timestamp & 0xFF000000) >> 24;
                    ack_payload[++ack_datasize] = (sensor_timestamp & 0x00FF0000) >> 16;
                    ack_payload[++ack_datasize] = (sensor_timestamp & 0x0000FF00) >> 8;
                    ack_payload[++ack_datasize] = sensor_timestamp & 0x000000FF;
                }
                
                if (df1 & DF1_BATT_VOLTAGE){
                    ack_payload[++ack_datasize] = (adc1_value & 0xFF00) >> 8;
                    ack_payload[++ack_datasize] = adc1_value & 0x00FF;
                }
                
                if (df1 & DF1_MOTOR_1234){
                    ack_payload[++ack_datasize] = motor_throttles[0];
                    ack_payload[++ack_datasize] = motor_throttles[1];
                    ack_payload[++ack_datasize] = motor_throttles[2];
                    ack_payload[++ack_datasize] = motor_throttles[3];
                }

                if (df1 & DF1_DIST){
                	short dist_mm = hc_sr04_Get_Dist(1) + 0.5;
                	if (dist_mm == -3){  // Out of measure range (5000/5.88 mm)
                		ack_payload[++ack_datasize] = 0xFF;
                		ack_payload[++ack_datasize] = 0xFF;
                	} else if (dist_mm == -2){ // 10ms time out
                		ack_payload[++ack_datasize] = 0xFE;
                		ack_payload[++ack_datasize] = 0xFE;
                	} else {
                		ack_payload[++ack_datasize] = (dist_mm & 0xFF00) >> 8;
                		ack_payload[++ack_datasize] = dist_mm & 0x00FF;
                	}
                }

                if (cur_bt_packet.payload_size > 2){
                	uint8_t df2 = cur_bt_packet.payload[2];
                	if (df2 & DF2_MPU_STATUS){
                		ack_payload[++ack_datasize] = mpu_status;
                	}

                	if (df2 & DF2_NRF24_STATUS){
                		ack_payload[++ack_datasize] = nrf24_status;
                	}

    	            if (df2 & DF2_SYSTICK){
    	                ack_payload[++ack_datasize] = (SysTick_ts_us & 0xFF000000) >> 24;
    	                ack_payload[++ack_datasize] = (SysTick_ts_us & 0x00FF0000) >> 16;
    	                ack_payload[++ack_datasize] = (SysTick_ts_us & 0x0000FF00) >> 8;
    	                ack_payload[++ack_datasize] = SysTick_ts_us & 0x000000FF;
    	            }
                }

                ack_payload[0] = ack_datasize;
                bt_SendACKWithPayload_Send();
                break;
            }
            case MOTOR_CTRL_CMD:{
                motor_throttles[1] = cur_bt_packet.payload[1];
                motor_throttles[2] = cur_bt_packet.payload[2];
                motor_throttles[3] = cur_bt_packet.payload[3];
                motor_throttles[4] = cur_bt_packet.payload[4];
                if (motor_throttles[0] == 0xBB){
                    TIM5_PWM_Set_Throttle(1, motor_throttles[1]);
                    TIM5_PWM_Set_Throttle(2, motor_throttles[2]);
                    TIM5_PWM_Set_Throttle(3, motor_throttles[3]);
                    TIM5_PWM_Set_Throttle(4, motor_throttles[4]);
                }
                ack_payload[0] = 0; // Send back an ACKDONE Msg
                break;
            }
            /*case ACTION_CTRL_CMD:{

            }*/
            default:
                
                break;
        }
        return 0;
    }
#elif defined(USE_NRF24_CONTROL)
    #define ACK_NOT_CMD                 0x00
    #define CMD_NOP                     0x00
    #define ACK_NOP                     0x80

    #define CMD_GET_MPU_DATA            0x01
    #define CMD_GET_MPU_DF_YPR          0x01
    #define CMD_GET_MPU_DF_GYRO         0x02
    #define CMD_GET_MPU_DF_ACCEL        0x04
    #define CMD_GET_MPU_COMP            0x08
    #define ACK_GET_MPU_DATA            0x81
    #define ACK_GET_MPU_DATA_VALID      0x02
    #define ACK_GET_MPU_CDATA_VALID     0x04

    #define CMD_GET_DIST_VOL            0x02
    #define CMD_GET_DV_DF_VOL           0x01
    #define CMD_GET_DV_DF_DIST          0x02
    #define ACK_GET_DIST_VOL            0x82

    #define CMD_SET_YPR                 0x10
    #define ACK_SET_YPR                 0x90
    #define CMD_SET_ACCEL_XYZ           0x11
    #define ACK_SET_ACCEL_XYZ           0x91
    #define CMD_SET_BASE_TH             0x12
    #define ACK_SET_BASE_TH             0x92

    #define CMD_ACT_HOVER               0x20
    #define ACK_ACT_HOVER               0xA0

    #define CMD_ACT_LAND                0x21
    #define ACK_ACT_LAND                0xA1

    extern unsigned long nrf24_packet_last_ts;

    void nrf24_packet_handler(uint8_t *packet, uint8_t size, uint8_t *ack, uint8_t *pACKsize){
        if (size == 0) {
            *pACKsize = 0;
            return ;
        }
        
        switch(packet[0]){
            case CMD_NOP:
                ack[0] = ACK_NOP; *pACKsize = 1;
                return ;
            case CMD_GET_MPU_DATA:
                if (size == 2) {
                    uint8_t ack_index = 1;
                    if (packet[1] & CMD_GET_MPU_DF_YPR){      // yaw + pitch + roll (*100); 2Bytes * 3
                        uint16_t data = 0;
                        data = (uint16_t) (yaw * 100);
                        ack[ack_index++] = (data >> 8) & 0xFF;
                        ack[ack_index++] = data & 0xFF;
                        data = (uint16_t) (pitch * 100);
                        ack[ack_index++] = (data >> 8) & 0xFF;
                        ack[ack_index++] = data & 0xFF;
                        data = (uint16_t) (roll * 100);
                        ack[ack_index++] = (data >> 8) & 0xFF;
                        ack[ack_index++] = data & 0xFF;
                    }
                    
                    if (packet[1] & CMD_GET_MPU_DF_GYRO){      // Gyro XYZ; 2Bytes * 3
                        ack[ack_index++] = (gyro[0] >> 8) & 0xFF;
                        ack[ack_index++] = gyro[0] & 0xFF;
                        ack[ack_index++] = (gyro[1] >> 8) & 0xFF;
                        ack[ack_index++] = gyro[1] & 0xFF;
                        ack[ack_index++] = (gyro[2] >> 8) & 0xFF;
                        ack[ack_index++] = gyro[2] & 0xFF;
                    }
                    
                    if (packet[1] & CMD_GET_MPU_DF_ACCEL){      // Accel XYZ; 2Bytes * 3
                        ack[ack_index++] = (accel[0] >> 8) & 0xFF;
                        ack[ack_index++] = accel[0] & 0xFF;
                        ack[ack_index++] = (accel[1] >> 8) & 0xFF;
                        ack[ack_index++] = accel[1] & 0xFF;
                        ack[ack_index++] = (accel[2] >> 8) & 0xFF;
                        ack[ack_index++] = accel[2] & 0xFF;
                    }
                    
                    if (packet[1] & CMD_GET_MPU_COMP){      // COMPASS XYZ; 2Bytes * 3
                        ack[ack_index++] = (compass_xyz[0] >> 8) & 0xFF;
                        ack[ack_index++] = compass_xyz[0] & 0xFF;
                        ack[ack_index++] = (compass_xyz[1] >> 8) & 0xFF;
                        ack[ack_index++] = compass_xyz[1] & 0xFF;
                        ack[ack_index++] = (compass_xyz[2] >> 8) & 0xFF;
                        ack[ack_index++] = compass_xyz[2] & 0xFF;
                    }
                    
                    ack[0] = ACK_GET_MPU_DATA | (mpu_data_valid == 0? ACK_GET_MPU_DATA_VALID : 0x00) | 
                                                (hmc_data_valid == 0? ACK_GET_MPU_CDATA_VALID : 0x00);
                    *pACKsize = ack_index;
                    return ;
                }
                break;
            case CMD_GET_DIST_VOL:
                if (size == 2) {
                    uint8_t ack_index = 1;
                    if (packet[1] & CMD_GET_DV_DF_VOL) {
                        ack[ack_index ++] = (adc1_value >> 8) & 0xFF;
                        ack[ack_index ++] = adc1_value & 0xFF;
                    }
                    if (packet[1] & CMD_GET_DV_DF_DIST) {
                        uint16_t data = hc_sr04_Get_Dist(1);        // Here, 10ms delay may happen
                        ack[ack_index ++] = (data >> 8) & 0xFF;
                        ack[ack_index ++] = data & 0xFF;
                    }
                    ack[0] = ACK_GET_DIST_VOL;
                    *pACKsize = ack_index;
                    return;
                }
                break;
            case CMD_SET_YPR:
                if (size == 7){
                    desiredYaw = ((((int) packet[1]) << 8) | packet[2]) / 100.0F;
                    desiredPitch = ((((int) packet[3]) << 8) | packet[4]) / 100.0F;
                    desiredRoll = ((((int) packet[5]) << 8) | packet[6]) / 100.0F;
                    ack[0] = ACK_SET_YPR; *pACKsize = 1;
                    pilot_cmd_desire = CMD_SET_YPR;
                    return ;
                }
                break;
            case CMD_SET_ACCEL_XYZ:
                if (size == 7){
                    desiredAX = (((short) packet[1]) << 8) | packet[2];
                    desiredAY = (((short) packet[3]) << 8) | packet[4];
                    desiredAZ = (((short) packet[5]) << 8) | packet[6];
                    ack[0] = ACK_SET_ACCEL_XYZ; *pACKsize = 1;
                    pilot_cmd_desire = CMD_SET_ACCEL_XYZ;
                    return ;
                }
                break;
            case CMD_SET_BASE_TH:
                if (size == 5) {
                    base_th[0] = packet[1]; base_th[1] = packet[2];
                    base_th[2] = packet[3]; base_th[3] = packet[4];
                    ack[0] = ACK_SET_BASE_TH; *pACKsize = 1;
                    //iored_printf("SET BASE: %d\r\n", packet[4]);
                    return ;
                }
                break;
            case CMD_ACT_HOVER:
                if (size == 1){
                    ack[0] = ACK_ACT_HOVER; *pACKsize = 1;
                    pilot_cmd_act = CMD_ACT_HOVER;
                    return ;
                } else if (size == 3){
                    desireDist = (((uint16_t) packet[1]) << 8) | packet[2];
                    ack[0] = ACK_ACT_HOVER; *pACKsize = 1;
                    pilot_cmd_act = CMD_ACT_HOVER;
                    return ;
                }
                break;
            case CMD_ACT_LAND:
                if (size == 1) {
                    ack[0] = ACK_ACT_LAND; *pACKsize = 1;
                    pilot_cmd_act = CMD_ACT_LAND;
                    return ;
                }
                break;
            default:
                ack[0] = ACK_NOT_CMD; *pACKsize = 1;
                return ;
        }

        ack[0] = ACK_NOT_CMD; *pACKsize = 1;
        return ;
    }

#endif

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    int result;
//	uint8_t i = 0;
//    uint8_t d[6] = {0, 0, 0, 0, 0};
    unsigned long curr_time = 0;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	// Setup SysTick for Delay function calls
	SysTick_Setup();
	// Setup TIM5 4 Channels for PWM signals
	TIM5_PWM_Setup();
	// Setup EXTI Interrupt (Callback Functions for nRF24(SPI) and MPU6050(I2C) have not yet be registered yet)
	EXTI_Setup();

#ifdef USE_IO_RED
	// Setup USART1 (For IO Rediretion for temporary Debug purpose)
	USART_IO_ReD_Setup();
#endif
	
    // MPU6050 I2C Initialization
	MPU6050_I2C_Setup();
	result = mpu_init();
    if (!result) {
        if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) result |= 0x01;
        if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) result |= 0x02;
        if (mpu_set_sample_rate(DEFAULT_MPU_HZ)) result |= 0x04;
		if (mpu_set_compass_sample_rate(DEFAULT_MPU_HZ)) result |= 0x08;
		if (mpu_set_lpf(42)) result |= 0x10;
        if (dmp_load_motion_driver_firmware()) result |= 0x20;
        if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) result |= 0x40;
        if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL)) result |= 0x80;
        if (dmp_set_fifo_rate(DEFAULT_MPU_HZ)) result |= 0x100;
    	if (!result) {
    		iored_printf("MPU6050: Init Good\r\n");
    		mpu_status |= MPU_INIT_GOOD;	// 0x80
    	} else {
    		iored_printf("MPU6050: Init Failed (%X)\r\n", result);
    	}

        result = run_self_test();
        mpu_status |= result;
        if ( (result & 0x07) != 0x07 ){
        	iored_printf("MPU6050: ST Failed (%X)\r\n", result);
        }

        if(!mpu_set_dmp_state(1)){
            iored_printf("MPU6050: DMP Good\r\n");
            mpu_status |= MPU_DMP_GOOD;		// 0x40
        } else {
            iored_printf("MPU6050: DMP Fail\r\n");
        }
    }
	// Register MPU6050 EXTI callback
	EXTI_IRQnHanlder_RegisterCB(EXTI_Line15, mpu_exti_handler_cb);
	
#ifdef USE_NRF24_CONTROL
	// Setup SPI for NRF24
	SPI_NRF24_Setup();
	// Power Down NRF24L01 and Register the Call back function
	nRF24_PowerDown();
	
	if (NRF24_FAIL != nRF24_Check())
		nrf24_status |= NRF24_CHECK_GOOD; // 0x80

    if (nrf24_status & NRF24_CHECK_GOOD){
        iored_printf("NRF24 CHECK GOOD\r\n");
    } else {
        iored_printf("NRF24 CHECK FAILED\r\n");
    }
    
	nRF24_Config();
    nRF24_FlushTX();
    nRF24_FlushRX();
    nRF24_PowerUp();
    delay_ms(5);

    nRF24_GotoRXMode();
    EXTI_IRQnHanlder_RegisterCB(EXTI_Line8, nRF24_exti_handler_cb);
#endif

	// Setup Distance Sensor
	hc_sr04_Setup();
	// Setup UART4 for Bluetooth, TEMP solution as it conflicts with SDIO interface
#ifdef USE_BT_CONTROL
	bt_UART_Setup();
    bt_Register_PacketCB(bt_packet_handler);
#endif
	// Setup ADC for Voltage (This must be put later then the I2C, SPI setup
	// [NOTE] I2C, SPI settings above are using polling, the Setup following will cause I2C/SPI read/write procedure
	// 		broken, where later the data transfer via I2C/SPI resides in the respective interrupt handler
	//		Setting preemp*** priority will prevent I2C/SPI read/write get broken [TODO]
	//voltage_ADC_Setup();
	while (1){
		// Current DOF Status
		// 		float yaw, pitch, roll;			// 
		//		short compass_xyz[3];			// Heading [TODO] I don't care for this now
		//		short gyro[3], accel[3];		// Gyro/Accel [TODO] How to use?
		//		dist_mm: hc_sr04_Get_Dist()		// Mannually called to get distance(mm) max delay (10ms)
		// 		int16_t adc1_value = -1;		// Automitically updated by ADC ISR [Critical]
		// Output:
		// 	motor_throttles[5] = {0xFF, 0x00, 0x00, 0x00, 0x00};
		//  First Byte: Flag for who is controlling the motors
		//  1-4: Values of the throttles for 4 channels of motors
		// Pilot Input:
		//	Action : What do you want to do ?
		//		Hover?
		//		Go Left/Right?
		//		Roll Over?
		// Critical Situation:
		//	Battery Low
		//	BT Connection Lost
        if (adc1_value < BATT_V_CRITICAL){
            // battery low, go to land procedure
            // land()
        }
        else {
            
        }

        {// TEST
            int t = 199;
            if (fgetc(NULL) == 'S'){
                for (t = 0; t < 200; t ++){
                    TIM_SetCompare1(TIM5, 199+t);
                    iored_printf("%d\n", t);
                    delay_msus(1000, 0);
                }
                iored_printf("END\n");
            }
        }
#ifdef USE_NRF24_CONTROL
        if (nrf24_is_connection_dead(200)){
            get_ms(&curr_time);
        	//iored_printf("%lu(ms) NRF24 Connection Dead\r\n", curr_time);
        }
#endif
        
#ifdef USE_BT_CONTROL
		if (bt_isConnectionDead()){
            // BT conn dead, hover now
        }
#endif
        
        
	};
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */

	printf("Something Wrong Here: file %s on line %d\r\n", file, line)
	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
