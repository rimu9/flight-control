#include "MPU6500_spi.h"


float MPU65xx_GYRO_FIRSTRUN_BUFF[3];
int MPU65xx_GYRO_FIRSTRUN_CENT=0;

int16_t  MPU65xx_ACCL_FIFO[3][9];
int16_t  MPU65xx_GYRO_FIFO[3][9];

/**************************????********************************************
	   void  MPU65xx_newValues(int16_t x,int16_t y,int16_t z)
����:	  ��ֵ�˲�������FIFO����
����:  ����ADCֵ
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
����:	  ��ֵ�˲�������FIFO����
����:  ����ADCֵ
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
 * ��������MPU6500_Write_Reg
 * ����  ��SPIд��Ĵ���
 * ����  ��reg:ָ���ļĴ�����ַ��value��д���ֵ
 * ���  ��status������״ֵ̬
 */ 
u8 MPU6500_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI1_CS1_Enable;  										//ʹ��SPI����
	status = SPI1_Read_Write_Byte(reg); //����д����+�Ĵ�����
	SPI1_Read_Write_Byte(value);				//д��Ĵ���ֵ
	SPI1_CS1_Disable;    										//��ֹMPU9500
	return(status);											//����״ֵ̬
}

/*
 * ��������MPU6500_Read_Reg
 * ����  ��SPI��ȡ�Ĵ���
 * ����  ��reg:ָ���ļĴ�����ַ
 * ���  ��reg_val��reg�Ĵ�����ַ��Ӧ��ֵ
 */ 
u8 MPU6500_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI1_CS1_Enable;  										//ʹ��SPI����
	SPI1_Read_Write_Byte(reg | 0x80); 	//���Ͷ�����+�Ĵ�����
	reg_val = SPI1_Read_Write_Byte(0xff); //��ȡ�Ĵ���ֵ
	SPI1_CS1_Disable;  									  //��ֹMPU6500
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
 *����������������adc����--------��ʱ124us
*/
void MPU6500_spi_ReadALL(struct Sense_Data *SensorTemp)
{

	uint8_t i;
	u8 mpu6500_buf[14];
	int16_t accdata[3],gyrodata[3];
	
	SPI1_CS1_Enable; 																//ʹ��SPI����
	SPI1_Read_Write_Byte(ACCEL_XOUT_H|0x80); 				//�Ӽ��ٶȼƵļĴ�����ʼ���ж�ȡ�����Ǻͼ��ٶȼƵ�ֵ//���Ͷ�����+�Ĵ�����
	for(i	=	0;i	<	14;i++)														//һ����ȡ14�ֽڵ�����
	{
		mpu6500_buf[i]	=	SPI1_Read_Write_Byte(0xff);	//����0xff,��Ϊslave��ʶ��
	}	
	accdata[0]=(int16_t)((mpu6500_buf[0] << 8) | mpu6500_buf[1]) ;
	accdata[1]=(int16_t)((mpu6500_buf[2] << 8) | mpu6500_buf[3]) ;
	accdata[2]=(int16_t)((mpu6500_buf[4] << 8) | mpu6500_buf[5]) ;
	gyrodata[0]=(int16_t)((mpu6500_buf[8] << 8) | mpu6500_buf[9]) ;
	gyrodata[1]=(int16_t)((mpu6500_buf[10] << 8) | mpu6500_buf[11]) ;
	gyrodata[2]=(int16_t)((mpu6500_buf[12] << 8) | mpu6500_buf[13]) ;
	SPI1_CS1_Disable;  	    //��ֹSPI����
	
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
//		//��ͨ�˲���-1
	SensorTemp->accl_filtered_x=LPF2pApply_1(SensorTemp->accl_origin_x);
	SensorTemp->accl_filtered_y=LPF2pApply_2(SensorTemp->accl_origin_y);
	SensorTemp->accl_filtered_z=LPF2pApply_3(SensorTemp->accl_origin_z);
	SensorTemp->gyro_filtered_x=LPF2pApply_4(SensorTemp->gyro_origin_x-SensorTemp->gyro_quiet_x);
	SensorTemp->gyro_filtered_y=LPF2pApply_5(SensorTemp->gyro_origin_y-SensorTemp->gyro_quiet_y);
	SensorTemp->gyro_filtered_z=LPF2pApply_6(SensorTemp->gyro_origin_z-SensorTemp->gyro_quiet_z);
//	
//	//���������˲�-2
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
 *�������������������ϵ�mpu6500�Ƿ����
*/
void MPU6500_spi_Check(void)
{
	u8 id;
	id=MPU6500_Read_Reg(WHO_AM_I);		//��ȷ��ȡ��6500�ĵ�ַ
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
 *��������ʼ��mpu6500 ���̵�����
*/
void MPU6500_spi_Init(void)
{
	MPU6500_spi_Check();
	
	MPU6500_Write_Reg(PWR_MGMT_1,0X80);   		//��Դ����,��λMPU6500
	delay_ms(100);
	MPU6500_Write_Reg(SIGNAL_PATH_RESET,0X07);//�����ǡ����ٶȼơ��¶ȼƸ�λ
	delay_ms(100);
	MPU6500_Write_Reg(PWR_MGMT_1,0X01);   		//ѡ��ʱ��Դ
	MPU6500_Write_Reg(PWR_MGMT_2,0X00);   		//ʹ�ܼ��ٶȼƺ�������
	MPU6500_Write_Reg(CONFIG,0X02);						//��ͨ�˲��� 0x02 92hz (3.9ms delay) fs=1khz
	MPU6500_Write_Reg(SMPLRT_DIV,0X00);				//������1000/(1+0)=1000HZ
	MPU6500_Write_Reg(GYRO_CONFIG,0X18);  		//�����ǲ�����Χ 0X18 ����2000��
	MPU6500_Write_Reg(ACCEL_CONFIG,0x10); 		//���ٶȼƲ�����Χ 0X00 ����8g
	MPU6500_Write_Reg(ACCEL_CONFIG2,0x00);		//���ٶȼ�����1khz �˲���460hz (1.94ms delay)
	
	printf("mpu6500 set completed...");
}
