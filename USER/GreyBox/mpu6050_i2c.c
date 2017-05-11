#include "mpu6050_i2c.h"
#include "mpu6050_reg_def.h"
#include <stdio.h>

/**
 * 	@brief Initialize the I2C Peripheral used by the MPU6050 sensor
 *	MPU6050_I2C_PERIPH_RCC, MPU6050_I2C_GPIO, MPU6050_I2C_PORT
 *	are defined in mpu6050_i2c.h
 *  WARNING: Change RCC_APB* calls accordingly if the above macros are changed
 *			For STM32F103 I2C2 is used by now, so it's okay
 *	
 *	@return MPU6050_FAILED in case init fail, otherwise MPU6050_SUCCESS
 */
uint8_t MPU6050_I2C_Init(void){
	RCC_APB1PeriphClockCmd(MPU6050_I2C_PERIPH_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(MPU6050_I2C_GPIO, ENABLE);
	
	I2C_Cmd(MPU6050_I2C_PORT, ENABLE); // Enable the I2C1
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	I2C_Config(MPU6050_I2C_PORT); // MPU6050 is connected to the I2C1 PB.6&7
	
	/* Check the "Who Am I" Register*/
	if (MPU6050_Read_Reg(MPU6050_REG_WHOAMI) != MPU6050_MY_ID)
		return MPU6050_FAILED;
	else
		return MPU6050_SUCCESS;
}

/**
 * @brief Reset/Wake Chip
 * @param[in] bEnable MPU6050_ENABLE/MPU6050_DISABLE
 */
void MPU6050_Reset(uint8_t bEnable){
	MPU6050_Write_Bits(MPU6050_RA_PWR_MGMT1, bEnable, MPU6050_RA_PWR_MGMT1_DEV_RST_BIT, 1);
}

void MPU6050_Set_Bypass(uint8_t bEnable){
	if (bEnable == MPU6050_ENABLE){
		// Disable the AUX I2C IF first, just in case
		MPU6050_Write_Bits(MPU6050_RA_USER_CTRL, MPU6050_DISABLE, MPU6050_RA_USER_CTRL_I2C_MST_EN_BIT, 1);
		DELAY_MS(3);
		// Enable the Bypass Mode
		MPU6050_Write_Bits(MPU6050_RA_INT_PIN_CFG, MPU6050_ENABLE, MPU6050_RA_INT_PIN_CFG_I2C_BYPASS_EN_BIT, 1);
	} else {
		MPU6050_Write_Bits(MPU6050_RA_INT_PIN_CFG, MPU6050_DISABLE,	MPU6050_RA_INT_PIN_CFG_I2C_BYPASS_EN_BIT, 1);
	}
}

/**
 *	@param[in] bLvl MPU6050_INT_L_H/L Interrupt Active High/Low
 */
void MPU6050_Set_IntLvl(uint8_t bLvl){
	MPU6050_Write_Bits(MPU6050_RA_INT_PIN_CFG, bLvl, MPU6050_RA_INT_PIN_CFG_INT_LVL_BIT, 1);
}

/**
 *	@param[in] bEnable MPU6050_DISABLE : 50us Pulse
 *						MPU6050_ENABLE : Active till cleared by RD
 */
void MPU6050_Set_IntLatch(uint8_t bEnable){
	MPU6050_Write_Bits(MPU6050_RA_INT_PIN_CFG, bEnable, MPU6050_RA_INT_PIN_CFG_LATCH_INT_EN_BIT, 1);
}

/**
 *	@param[in] bRDC_by MPU6050_RDC_ONLY 0 : Int Clear by Read INT_STATUS ONLY
 *						  MPU6050_RDC_ANY  1 : Int Clear by ANY Read
 */ 
void MPU6050_Set_IntRDClear(uint8_t bRDC_by){
	MPU6050_Write_Bits(MPU6050_RA_INT_PIN_CFG, bRDC_by, MPU6050_RA_INT_PIN_CFG_INT_RD_CLEAR_BIT, 1);
}

/**
 *	@brief [TODO] The meaning of this function is not clear by far
 */
void MPU6050_Get_AccelProdShift(float *st_shift){
	uint8_t tmp[4], shift_code[3], ii;
	MPU6050_Burst_Read(MPU6050_RA_ACCEL_SHIFT,temp,4);

	// Code from the eMPL driver
	shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
    for (ii = 0; ii < 3; ii++) {
        if (!shift_code[ii]) {
            st_shift[ii] = 0.f;
            continue;
        }
        /* Equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[ii] = 0.34f;
        while (--shift_code[ii])
            st_shift[ii] *= 1.034f;
    }

    return ;
}

/**
 *	@brief [TODO] Meaning/Usage of the following two selftest function need to figure about later
 */
int MPU6050_SelfTest_Accel(long *bias_regular, long *bias_st){
	int jj, result = 0;
    float st_shift[3], st_shift_cust, st_shift_var;

    MPU6050_Get_AccelProdShift(st_shift);
    for(jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (st_shift[jj]) {
            st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
            if (fabs(st_shift_var) > test.max_accel_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < test.min_g) ||
            (st_shift_cust > test.max_g))
            result |= 1 << jj;
    }

    return result;
}

int MPU6050_SelfTest_Gyro(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    unsigned char tmp[3];
    float st_shift, st_shift_cust, st_shift_var;

    if (i2c_read(st.hw->addr, 0x0D, 3, tmp))
        return 0x07;

    tmp[0] &= 0x1F;
    tmp[1] &= 0x1F;
    tmp[2] &= 0x1F;

    for (jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (tmp[jj]) {
            st_shift = 3275.f / test.gyro_sens;
            while (--tmp[jj])
                st_shift *= 1.046f;
            st_shift_var = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var) > test.max_gyro_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < test.min_dps) ||
            (st_shift_cust > test.max_dps))
            result |= 1 << jj;
    }
    return result;
}

//[TODO] Functions for secondary magnetic sensor selftest and configuration need to be added

int MPU6050_Get_SelfTestBiases(long *gyro, long *accel, uint8_t hw_test){

	
}

int MPU6050_Setup(void){
	uint8_t data[6], rev;

	MPU6050_Reset(MPU6050_ENABLE);
	// [TODO] Delay 100ms
	MPU6050_Reset(MPU6050_DISABLE);
	// Check product revision
	MPU6050_Burst_Read(MPU6050_RA_XA_OFFS_H,data,6);
	rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) |
        (data[1] & 0x01);
    if (rev) {

    }
}

int MPu6050_Set_Gyro_FSR(unsigned short fsr)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 250:
        data = INV_FSR_250DPS << 3;
        break;
    case 500:
        data = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.gyro_fsr == (data >> 3))
        return 0;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data))
        return -1;
    st.chip_cfg.gyro_fsr = data >> 3;
    return 0;
}