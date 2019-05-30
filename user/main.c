/***********************************************************************

����stm32f103rct6�����Ʒɿ�
��������
      mpu6500 -----spiͨѶ---SPI1
			hmc5983 -----spiͨѶ---SPI1
      ms5611  -----spiͨѶ---SPI1    PA5 PA6 PA7

			GPSģ�� -----usart����ͨѶ----usart2   PA2 PA3 
			������ģ��--

			�����ô��� ----����1----PA9 PA10

************************************************************************/

#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x_it.h"
#include "IIC.h"
#include "math.h"
#include "USART1.h"
#include "stdio.h"
#include "SYS_FUN.h"
#include "Tim.h"
#include "motor.h"
#include "filter.h"
#include "delay.h"
#include "usart2.h"
#include "gpio.h"
#include "public.h"
#include "HMC5983_spi.h"
#include "MPU6500_spi.h"
#include "U_100.h"
#include "GPS.h"
#include "attitude_estimator_q.h"

float data_mpu6050[8];
//float dt,t1;
//uint8_t ucRxBuffer[11];
//int ucRxCnt = 0;	
//int makesure=0,start=0;
extern uint8_t ucRxBuffer[9];
char FLAG=0;
extern char RecBuff[76];


char str[100]; //printf������ַ�����
struct Sense_Data senser_datas;
struct System_Status system_status;
Quaternion q,q_d;  //q:��ǰ��̬��Ԫ�أ�q_d:������̬��Ԫ��

int main()
{
	SystemClock_HSE(9);           //��ʼ��ϵͳʱ��Ϊ8*9=72MHz;
  cycleCounterInit();				//�ó���ǰϵͳʱ��Ƶ��
  SysTick_Config(SystemCoreClock / 1000);	//SysTick��׼��ʱ,1ms

	UART1_init(SysClock,1200);   //����1��ʼ��
  USART2_Init(38400);  //GPS��
  //MotorInit();  //���PWM�ڳ�ʼ��
	//IIC_Init();	
	SPI1_Init();
	TIM4_Init(SysClock,1000);//��ʱ��ʱ�� ����1ms
	NVIC_INIT();	    //�жϳ�ʼ��
	
	//MPU65xx_Init();  //mpu6500��ʼ��
	//HMC5883L_Init();  //�����Ƴ�ʼ��
//	MS5611_Init();   //��ѹ�Ƴ�ʼ��
	
	MPU6500_spi_Init();	
	system_status.mpu65xx_first_run=1;
	MPU6500_Get_QuietData(&senser_datas);
//	
  HMC5983_spi_init();
//	MS5611_SPI_Init();
//	HC_SR04_Init(); //������ģ�����ų�ʼ��
	
  Low_Passfilter_Init(); //��ͨ�˲�����ʼ��
	
	
	
	system_status.sensor_update_lock=0;
	system_status.GPS_alive=0;
  while(1)  
  {
		
		loop();
		
/***********************************************************************************************************************************************************************************************************
***********************************************************����Ϊ���Ժ������ܵĲ��֣���������whileѭ�������ݣ���Ϊ����ʱʹ��********************************************************************************
***********************************************************************************************************************************************************************************************************/		
		
		
//		GPS_PVT_Parse();
//		delay_ms(10);
//		data_mpu6050[0]=(double)GPS_Yaw;
//		data_mpu6050[1]=Longitude;
//		data_mpu6050[2]=Latitude;
//		USART1_SendData((unsigned char *)(&data_mpu6050),sizeof(data_mpu6050));
		
//	  MPU6050AccRead(gyro);
//		gyro_filter[0]=LPF2pApply_4(gyro[0]);
//		gyro_filter[1]=LPF2pApply_5(gyro[1]);
//		gyro_filter[2]=LPF2pApply_6(gyro[2]);
//		//MPU6050GyroRead(gyro);
//	  for(i=0;i<3;i++)
//		{
//				data_mpu6050[i]=gyro[i];
//			  data_mpu6050[i+3]=gyro_filter[i];
//			 
//		}

//		MPU6500_spi_ReadALL(&senser_datas);
//		HMC5983_spi_Read_All(&senser_datas);
////		 t1=micros();
//		//_init();
//////	ddt=micros()-t1;	
//////		USART1_SendData((unsigned char *)(&ddt),sizeof(ddt));
//////		theta=atan2((2*(q.q1*q.q2+q.q0*q.q3)),(1-2*(q.q2*q.q2+q.q3*q.q3)));
//////		USART1_SendData((unsigned char *)(&theta),sizeof(theta));
//		data_mpu6050[0]=senser_datas.gyro_filtered_x;
//		data_mpu6050[1]=senser_datas.gyro_filtered_y;
//		data_mpu6050[2]=senser_datas.gyro_filtered_z;
//		data_mpu6050[3]=senser_datas.mag_filtered_x;
//		data_mpu6050[4]=senser_datas.mag_filtered_y;
//		data_mpu6050[5]=senser_datas.accl_filtered_x;
//		data_mpu6050[6]=senser_datas.accl_filtered_y;
//		data_mpu6050[7]=senser_datas.accl_filtered_z;
//		USART1_SendData((unsigned char *)(data_mpu6050),sizeof(data_mpu6050));
//		delay_ms(10);
////		data_mpu6050[5]=senser_datas.gyro_origin_z;
//		MS5611_Get_Altittude();
//		MS5611_SPI_Get_Altittude();
//		USART1_SendData((unsigned char *)(&senser_datas.altitude_orign),sizeof(senser_datas.altitude_orign));
//		delay_ms(10);
		
		
//		t1=micros();
//		HMC5983_spi_Read_All(&senser_datas);
//		dt=micros()-t1; //�ӳ�2us
//		USART1_SendData((unsigned char *)(&dt),sizeof(dt));


//		data_mpu6050[0]=(float)((int16_t)ucRxBuffer[3]<<8|ucRxBuffer[2])/32768*180; 
//		data_mpu6050[1]=(float)((int16_t)ucRxBuffer[5]<<8|ucRxBuffer[4])/32768*180; 
//		data_mpu6050[2]=(float)((int16_t)ucRxBuffer[1]<<8|ucRxBuffer[0])/32768*180; 
		
//		data_mpu6050[3]=senser_datas.gyro_filtered_x;
//		data_mpu6050[4]=senser_datas.gyro_filtered_y;
//		data_mpu6050[5]=senser_datas.gyro_filtered_z;
//		
//		USART1_SendData((unsigned char *)(data_mpu6050),sizeof(data_mpu6050));
//		delay_ms(100);
//		HC_SR04_Start();
		
//		USART1_SendData((unsigned char *)(Ublox_Data),sizeof(Ublox_Data));
//		delay_ms(100);
		
//		MPU6500_spi_ReadALL(&senser_datas);
//		HMC5983_spi_Read_All(&senser_datas);
//		data_mpu6050[0]=(senser_datas.mag_filtered_x-22);
//		data_mpu6050[1]=(senser_datas.mag_filtered_y+149);
//		data_mpu6050[2]=(senser_datas.mag_filtered_z+20);
//		data_mpu6050[3]=senser_datas.mag_filtered_x;
//		data_mpu6050[4]=senser_datas.mag_filtered_y;
//		data_mpu6050[5]=senser_datas.mag_filtered_z;
//		USART1_SendData((unsigned char *)(data_mpu6050),sizeof(data_mpu6050));
//		delay_ms(100);
  }     
}
