#ifndef __PUBLIC_H
#define __PUBLIC_H

#include "stm32f10x.h"
#include "stm32f10x_it.h" 

struct Sense_Data{
	
	   float accl_origin_x;
		 float accl_origin_y;
	   float accl_origin_z;//accl ADCԭʼ����
	   float gyro_origin_x;
	   float gyro_origin_y;
	   float gyro_origin_z;//gyro ADCԭʼ����
		 float gyro_quiet_x;
	   float gyro_quiet_y;
	   float gyro_quiet_z;//���ٶȾ���
	   float mag_origin_x;
		 float mag_origin_y;
		 float mag_origin_z;//������ ADCԭʼ����
	
	   float accl_filtered_x;
		 float accl_filtered_y;
	   float accl_filtered_z;//���ٶȼƵ�ͨ�˲�������
	   float gyro_filtered_x;
	   float gyro_filtered_y;
	   float gyro_filtered_z;//���ٶȵ�ͨ�˲�������
	   float mag_filtered_x;
	   float mag_filtered_y;
	   float mag_filtered_z;//�����Ƶ�ͨ�˲�������
		 
		 float temperature;
	   float pressure_origin;
		 float pressure_filtered;
		 float altitude_orign;
		 float altitude_filtered;
		 float altitude_start_quiet;
            };
extern struct Sense_Data senser_datas;

						
						
struct System_Status{
		
			uint8_t mpu65xx_first_run; //mpu�ϵ������̬����־
			uint8_t ms5611_first_run;  //ms5611�ϵ������ɸ߶ȱ�־
	    uint8_t sensor_update_lock;       //��ѭ����ȡ����������ǰ��ֹ����ʹ��sensor_data�ṹ��������ݡ�
	    uint8_t attitude_estimator_q_init;
			uint8_t GPS_alive;
            };	
						
extern struct System_Status system_status;		
						
						
typedef struct
{
 float E;
 float N;
 float D;
}Vector3_NED;

typedef struct
{
 float x;
 float y;
 float z;
}Vector3f;
						

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}Quaternion;






#endif
