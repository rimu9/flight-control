/*    
功能：
1.飞机姿态传感器初始化
2.硬件上的数据中断int脚已连接，软件是轮训机制，大家可以在这方面做修改
------------------------------------
*/
#include "MPU6050.h"
#include "IIC.h"
#include "USART1.h"
#include "delay.h"
//#include "extern_variable.h"
//#include "config.h"


/*

#define MPU6000_ACCEL_DEFAULT_RANGE_G			8
#define MPU6000_ACCEL_DEFAULT_RATE			1000
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G			8
#define MPU6000_GYRO_DEFAULT_RATE			1000
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		42

#define MPU6000_ONE_G					9.80665f
*/


/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_trange)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_tenabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}



/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void) {

		IICwriteByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay_ms(50);
    IICwriteByte(devAddr, MPU6050_RA_SMPLRT_DIV, 0x00);      //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    IICwriteByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    IICwriteByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    IICwriteByte(devAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
		MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Accel scale 8g (4096 LSB/g)
    IICwriteByte(devAddr, MPU6050_RA_ACCEL_CONFIG, 2 << 3);

		printf("MPU6050 init success...\r\n");
}
//读acc
void MPU6050AccRead(int16_t *accData)       //int16_t = short int  
{
    uint8_t buf[6];

    IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);


}
//读gyro
void MPU6050GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    IICreadBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

