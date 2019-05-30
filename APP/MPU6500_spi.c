#include "MPU6500_spi.h"


float MPU65xx_GYRO_FIRSTRUN_BUFF[3];
int MPU65xx_GYRO_FIRSTRUN_CENT=0;

int16_t  MPU65xx_ACCL_FIFO[3][9];
int16_t  MPU65xx_GYRO_FIFO[3][9];

/**************************????********************************************
	   void  MPU65xx_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新ADC值
*******************************************************************************/
void  MPU65xx_ACCL_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<8;i++){
		MPU65xx_ACCL_FIFO[0][i-1]=MPU65xx_ACCL_FIFO[0][i];
		MPU65xx_ACCL_FIFO[1][i-1]=MPU65xx_ACCL_FIFO[1][i];
		MPU65xx_ACCL_FIFO[2][i-1]=MPU65xx_ACCL_FIFO[2][i];
	}

	MPU65xx_ACCL_FIFO[0][7]=x;
	MPU65xx_ACCL_FIFO[1][7]=y;
	MPU65xx_ACCL_FIFO[2][7]=z;

	sum=0;
	for(i=0;i<8;i++){	//??????????????
   		sum+=MPU65xx_ACCL_FIFO[0][i];
	}
	MPU65xx_ACCL_FIFO[0][8]=sum/10;	//??????

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU65xx_ACCL_FIFO[1][i];
	}
	MPU65xx_ACCL_FIFO[1][8]=sum/10;

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU65xx_ACCL_FIFO[2][i];
	}
	MPU65xx_ACCL_FIFO[2][8]=sum/10;
}


/**************************????********************************************
	   void  MPU65xx_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新ADC值
*******************************************************************************/
void  MPU65xx_GYRO_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<8;i++){
		MPU65xx_GYRO_FIFO[0][i-1]=MPU65xx_GYRO_FIFO[0][i];
		MPU65xx_GYRO_FIFO[1][i-1]=MPU65xx_GYRO_FIFO[1][i];
		MPU65xx_GYRO_FIFO[2][i-1]=MPU65xx_GYRO_FIFO[2][i];
	}

	MPU65xx_GYRO_FIFO[0][7]=x;
	MPU65xx_GYRO_FIFO[1][7]=y;
	MPU65xx_GYRO_FIFO[2][7]=z;

	sum=0;
	for(i=0;i<8;i++){	//??????????????
   		sum+=MPU65xx_GYRO_FIFO[0][i];
	}
	MPU65xx_GYRO_FIFO[0][8]=sum/10;	//??????

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU65xx_GYRO_FIFO[1][i];
	}
	MPU65xx_GYRO_FIFO[1][8]=sum/10;

	sum=0;
	for(i=0;i<8;i++){
   		sum+=MPU65xx_GYRO_FIFO[2][i];
	}
	MPU65xx_GYRO_FIFO[2][8]=sum/10;
} 


/*
 * 函数名：MPU6500_Write_Reg
 * 描述  ：SPI写入寄存器
 * 输入  ：reg:指定的寄存器地址；value：写入的值
 * 输出  ：status：返回状态值
 */ 
u8 MPU6500_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI1_CS1_Enable;  										//使能SPI传输
	status = SPI1_Read_Write_Byte(reg); //发送写命令+寄存器号
	SPI1_Read_Write_Byte(value);				//写入寄存器值
	SPI1_CS1_Disable;    										//禁止MPU9500
	return(status);											//返回状态值
}

/*
 * 函数名：MPU6500_Read_Reg
 * 描述  ：SPI读取寄存器
 * 输入  ：reg:指定的寄存器地址
 * 输出  ：reg_val：reg寄存器地址对应的值
 */ 
u8 MPU6500_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI1_CS1_Enable;  										//使能SPI传输
	SPI1_Read_Write_Byte(reg | 0x80); 	//发送读命令+寄存器号
	reg_val = SPI1_Read_Write_Byte(0xff); //读取寄存器值
	SPI1_CS1_Disable;  									  //禁止MPU6500
	return(reg_val);
}

void MPU6500_Get_QuietData(struct Sense_Data *SensorTemp)
{
	int i;
	for(i=0;i<50;i++)
	{
		MPU6500_spi_ReadALL(SensorTemp);
	}
}

/*
 *描述：读出传感器adc数据--------耗时124us
*/
void MPU6500_spi_ReadALL(struct Sense_Data *SensorTemp)
{

	uint8_t i;
	u8 mpu6500_buf[14];
	int16_t accdata[3],gyrodata[3];
	
	SPI1_CS1_Enable; 																//使能SPI传输
	SPI1_Read_Write_Byte(ACCEL_XOUT_H|0x80); 				//从加速度计的寄存器开始进行读取陀螺仪和加速度计的值//发送读命令+寄存器号
	for(i	=	0;i	<	14;i++)														//一共读取14字节的数据
	{
		mpu6500_buf[i]	=	SPI1_Read_Write_Byte(0xff);	//输入0xff,因为slave不识别
	}	
	accdata[0]=(int16_t)((mpu6500_buf[0] << 8) | mpu6500_buf[1]) ;
	accdata[1]=(int16_t)((mpu6500_buf[2] << 8) | mpu6500_buf[3]) ;
	accdata[2]=(int16_t)((mpu6500_buf[4] << 8) | mpu6500_buf[5]) ;
	gyrodata[0]=(int16_t)((mpu6500_buf[8] << 8) | mpu6500_buf[9]) ;
	gyrodata[1]=(int16_t)((mpu6500_buf[10] << 8) | mpu6500_buf[11]) ;
	gyrodata[2]=(int16_t)((mpu6500_buf[12] << 8) | mpu6500_buf[13]) ;
	SPI1_CS1_Disable;  	    //禁止SPI传输
	
	SensorTemp->accl_origin_x=(float)accdata[0]/4096;
	SensorTemp->accl_origin_y=(float)accdata[1]/4096;
	SensorTemp->accl_origin_z=(float)accdata[2]/4096;
	SensorTemp->gyro_origin_x=(float)gyrodata[0]/16.384;
	SensorTemp->gyro_origin_y=(float)gyrodata[1]/16.384;
	SensorTemp->gyro_origin_z=(float)gyrodata[2]/16.384;
	
	if(system_status.mpu65xx_first_run==1)
	{
		if(MPU65xx_GYRO_FIRSTRUN_CENT<50){
			MPU65xx_GYRO_FIRSTRUN_BUFF[0]+=SensorTemp->gyro_origin_x;
			MPU65xx_GYRO_FIRSTRUN_BUFF[1]+=SensorTemp->gyro_origin_y;
			MPU65xx_GYRO_FIRSTRUN_BUFF[2]+=SensorTemp->gyro_origin_z;
			MPU65xx_GYRO_FIRSTRUN_CENT++;
		}
		else{
			system_status.mpu65xx_first_run=0;
			senser_datas.gyro_quiet_x=MPU65xx_GYRO_FIRSTRUN_BUFF[0]/50;
			senser_datas.gyro_quiet_y=MPU65xx_GYRO_FIRSTRUN_BUFF[1]/50;
			senser_datas.gyro_quiet_z=MPU65xx_GYRO_FIRSTRUN_BUFF[2]/50;
		}
	}
	else
	{
//		//低通滤波器-1
	SensorTemp->accl_filtered_x=LPF2pApply_1(SensorTemp->accl_origin_x);
	SensorTemp->accl_filtered_y=LPF2pApply_2(SensorTemp->accl_origin_y);
	SensorTemp->accl_filtered_z=LPF2pApply_3(SensorTemp->accl_origin_z);
	SensorTemp->gyro_filtered_x=LPF2pApply_4(SensorTemp->gyro_origin_x-SensorTemp->gyro_quiet_x);
	SensorTemp->gyro_filtered_y=LPF2pApply_5(SensorTemp->gyro_origin_y-SensorTemp->gyro_quiet_y);
	SensorTemp->gyro_filtered_z=LPF2pApply_6(SensorTemp->gyro_origin_z-SensorTemp->gyro_quiet_z);
//	
//	//滑动窗口滤波-2
//	MPU6500_ACCL_newValues(accdata[0],accdata[1],accdata[2]);
//	MPU6500_GYRO_newValues(gyrodata[0],gyrodata[1],gyrodata[2]);
//	SensorTemp->accl_filtered_x=MPU65xx_ACCL_FIFO[0][8];
//	SensorTemp->accl_filtered_y=MPU65xx_ACCL_FIFO[1][8];
//	SensorTemp->accl_filtered_z=MPU65xx_ACCL_FIFO[2][8];
//	SensorTemp->gyro_filtered_x=MPU65xx_GYRO_FIFO[0][8];
//	SensorTemp->gyro_filtered_y=MPU65xx_GYRO_FIFO[1][8];
//	SensorTemp->gyro_filtered_z=MPU65xx_GYRO_FIFO[2][8];
	}
}


/*
 *描述：检测挂载在总线上的mpu6500是否存在
*/
void MPU6500_spi_Check(void)
{
	u8 id;
	id=MPU6500_Read_Reg(WHO_AM_I);		//正确读取到6500的地址
  if(id==0x70)
	{
		printf("mpu6500 found...");
	}
	else
	{
		printf("mpu6500 not found...");
	}
}

/*
 *描述：初始化mpu6500 量程等数据
*/
void MPU6500_spi_Init(void)
{
	MPU6500_spi_Check();
	
	MPU6500_Write_Reg(PWR_MGMT_1,0X80);   		//电源管理,复位MPU6500
	delay_ms(100);
	MPU6500_Write_Reg(SIGNAL_PATH_RESET,0X07);//陀螺仪、加速度计、温度计复位
	delay_ms(100);
	MPU6500_Write_Reg(PWR_MGMT_1,0X01);   		//选择时钟源
	MPU6500_Write_Reg(PWR_MGMT_2,0X00);   		//使能加速度计和陀螺仪
	MPU6500_Write_Reg(CONFIG,0X02);						//低通滤波器 0x02 92hz (3.9ms delay) fs=1khz
	MPU6500_Write_Reg(SMPLRT_DIV,0X00);				//采样率1000/(1+0)=1000HZ
	MPU6500_Write_Reg(GYRO_CONFIG,0X18);  		//陀螺仪测量范围 0X18 正负2000度
	MPU6500_Write_Reg(ACCEL_CONFIG,0x10); 		//加速度计测量范围 0X00 正负8g
	MPU6500_Write_Reg(ACCEL_CONFIG2,0x00);		//加速度计速率1khz 滤波器460hz (1.94ms delay)
	
	printf("mpu6500 set completed...");
}
