#include "MPU65xx.h"


int16_t  MPU6500_ACCL_FIFO[3][9];
int16_t  MPU6500_GYRO_FIFO[3][9];

/**************************????********************************************
	   void  MPU65xx_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新ADC值
*******************************************************************************/
void  MPU6500_ACCL_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<8;i++){
		MPU6500_ACCL_FIFO[0][i-1]=MPU6500_ACCL_FIFO[0][i];
		MPU6500_ACCL_FIFO[1][i-1]=MPU6500_ACCL_FIFO[1][i];
		MPU6500_ACCL_FIFO[2][i-1]=MPU6500_ACCL_FIFO[2][i];
	}

	MPU6500_ACCL_FIFO[0][7]=x;
	MPU6500_ACCL_FIFO[1][7]=y;
	MPU6500_ACCL_FIFO[2][7]=z;

	sum=0;
	for(i=0;i<8;i++){	//??????????????
   		sum+=MPU6500_ACCL_FIFO[0][i];
	}
	MPU6500_ACCL_FIFO[0][8]=sum/10;	//??????

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU6500_ACCL_FIFO[1][i];
	}
	MPU6500_ACCL_FIFO[1][8]=sum/10;

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU6500_ACCL_FIFO[2][i];
	}
	MPU6500_ACCL_FIFO[2][8]=sum/10;
} //HMC58X3_newValues


/**************************????********************************************
	   void  MPU65xx_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新ADC值
*******************************************************************************/
void  MPU6500_GYRO_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<8;i++){
		MPU6500_GYRO_FIFO[0][i-1]=MPU6500_GYRO_FIFO[0][i];
		MPU6500_GYRO_FIFO[1][i-1]=MPU6500_GYRO_FIFO[1][i];
		MPU6500_GYRO_FIFO[2][i-1]=MPU6500_GYRO_FIFO[2][i];
	}

	MPU6500_GYRO_FIFO[0][7]=x;
	MPU6500_GYRO_FIFO[1][7]=y;
	MPU6500_GYRO_FIFO[2][7]=z;

	sum=0;
	for(i=0;i<8;i++){	//??????????????
   		sum+=MPU6500_GYRO_FIFO[0][i];
	}
	MPU6500_GYRO_FIFO[0][8]=sum/10;	//??????

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU6500_GYRO_FIFO[1][i];
	}
	MPU6500_GYRO_FIFO[1][8]=sum/10;

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU6500_GYRO_FIFO[2][i];
	}
	MPU6500_GYRO_FIFO[2][8]=sum/10;
} //HMC58X3_newValues






/*************************
检测是否挂载mpu6500，读 WHO AM I 地址，返回具体型号
  mpu6500----0x70(此地址并非iic通讯的器件地址)
**************************/
void MPU65xx_Check()
{
	u8 IDtemp;
	//IDtemp=I2C_ReadOneByte(0xD0,0x75);
	IICreadBytes(0xd0,0x75,1,&IDtemp);
	printf("%c",IDtemp);
	
	if(IDtemp==0x70)
		printf("MPU65xx check success...\r\n");
	else
		printf("MPU65xx not found...\r\n");
}

/**************************????********
原型:		void MPU6050_initialize(void)
功能:	   初始化mpu6500进入使用状态
**************************************/
void MPU65xx_Init(void) {
	
	  MPU65xx_Check();

		IICwriteByte(MPU65xx_devAddr, MPU6050_RA_PWR_MGMT_1, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    delay_ms(50);
    IICwriteByte(MPU65xx_devAddr, MPU6050_RA_SMPLRT_DIV, 0x00);      //1000hz      --SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    IICwriteByte(MPU65xx_devAddr, MPU6050_RA_PWR_MGMT_1, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    IICwriteByte(MPU65xx_devAddr, MPU6050_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    IICwriteByte(MPU65xx_devAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
		IICwriteByte(MPU65xx_devAddr, MPU6050_RA_GYRO_CONFIG, 0x18);       //+- 2000 dps
    IICwriteByte(MPU65xx_devAddr, MPU6050_RA_ACCEL_CONFIG, 0x10);      // Accel scale 8g (4096 LSB/g)
    
	  printf("MPU65xx init success...\r\n");
}


//读accl
void MPU65xxAccRead(int16_t *accData)       //int16_t = short int  
{
    uint8_t buf[6];

    IICreadBytes(MPU65xx_devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);


}
//读gyro
void MPU65xxGyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    IICreadBytes(MPU65xx_devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

void MPU6xxx_ReadAll(struct Sense_Data *SensorTemp)
{
	int16_t acc[3],gyro[3];
	
	MPU65xxAccRead(acc);
	MPU65xxGyroRead(gyro);
	
	//归一化
	SensorTemp->accl_origin_x=(float)acc[0]/4096;
	SensorTemp->accl_origin_y=(float)acc[1]/4096;
	SensorTemp->accl_origin_z=(float)acc[2]/4096;
	SensorTemp->gyro_origin_x=(float)gyro[0]/16.384;
	SensorTemp->gyro_origin_y=(float)gyro[1]/16.384;
	SensorTemp->gyro_origin_z=(float)gyro[2]/16.384;
	
//	//低通滤波器-1
//	SensorTemp->accl_filtered_x=LPF2pApply_1(SensorTemp->accl_origin_x);
//	SensorTemp->accl_filtered_y=LPF2pApply_2(SensorTemp->accl_origin_y);
//	SensorTemp->accl_filtered_z=LPF2pApply_3(SensorTemp->accl_origin_z);
//	SensorTemp->gyro_filtered_x=LPF2pApply_4(SensorTemp->gyro_origin_x-SensorTemp->gyro_quiet_x);
//	SensorTemp->gyro_filtered_y=LPF2pApply_5(SensorTemp->gyro_origin_y-SensorTemp->gyro_quiet_y);
//	SensorTemp->gyro_filtered_z=LPF2pApply_6(SensorTemp->gyro_origin_z-SensorTemp->gyro_quiet_z);
//	
//	//滑动窗口滤波-2
//	MPU65xx_ACCL_newValues(acc[0],acc[1],acc[2]);
//	MPU65xx_GYRO_newValues(gyro[0],gyro[1],gyro[2]);
//	SensorTemp->accl_filtered_x=MPU6500_ACCL_FIFO[0][8];
//	SensorTemp->accl_filtered_y=MPU6500_ACCL_FIFO[1][8];
//	SensorTemp->accl_filtered_z=MPU6500_ACCL_FIFO[2][8];
//	SensorTemp->gyro_filtered_x=MPU6500_GYRO_FIFO[0][8];
//	SensorTemp->gyro_filtered_y=MPU6500_GYRO_FIFO[1][8];
//	SensorTemp->gyro_filtered_z=MPU6500_GYRO_FIFO[2][8];
	
}

