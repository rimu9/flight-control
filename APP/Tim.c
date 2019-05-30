#include "Tim.h"
#include "USART1.h"
#include "HMC5983_spi.h"
#include "MPU6500_spi.h"
#include "attitude_estimator_q.h"

uint8_t loop_cnt_10hz=0;
uint8_t loop_cnt_20hz=0;
uint8_t loop_cnt_50hz=0;
uint8_t loop_cnt_100hz=0;
uint8_t loop_cnt_200hz=0;


extern struct Sense_Data senser_datas;
extern Quaternion q,q_d;

void LOOP_10HZ(void)
{
	ANO_Data_Send_Status_USE_USB();
//		float qqt[8];
////	ti=micros();
//	qqt[0]=q.q0;qqt[1]=q.q1;qqt[2]=q.q2;qqt[3]=q.q3;qqt[4]=0;
//	qqt[5]=atan2(2*(q.q2*q.q3 + q.q0*q.q1),1-2*(q.q1*q.q1+q.q2*q.q2))*180/3.1415926;
//	qqt[6]=asin(-2*(q.q1*q.q3 - q.q0*q.q2))*180/3.1415926;
//	qqt[7]=atan2(2*(q.q1*q.q2 + q.q0*q.q3),1-2*(q.q2*q.q2 + q.q3*q.q3))*180/3.1415926;
//	USART1_SendData((unsigned char *)(qqt),sizeof(qqt));
//	dtt=micros()-ti;
}
void LOOP_20HZ(void)
{

}
void LOOP_50HZ(void)
{
	
}
void LOOP_100HZ(void)
{
	system_status.sensor_update_lock=1;
	MPU6500_spi_ReadALL(&senser_datas);
  HMC5983_spi_Read_All(&senser_datas);
	system_status.sensor_update_lock=0;
	
}
void LOOP_200HZ(void)
{
	attitude_estimator_q_main();
}

//主循环部分
void loop(void)
{
	if(loop_cnt_10hz>=99)
	{
		LOOP_10HZ();
		loop_cnt_10hz=0;
	}
	if(loop_cnt_20hz>=50)
	{
		LOOP_20HZ();
		loop_cnt_20hz=0;
	}
	if(loop_cnt_50hz>=20)
	{
		LOOP_50HZ();
		loop_cnt_50hz=0;
	}
	if(loop_cnt_100hz>=10)
	{
		LOOP_100HZ();
		loop_cnt_100hz=0;
	}
	if(loop_cnt_200hz>=5)
	{
		LOOP_200HZ();
		loop_cnt_200hz=0;
	}
}

void TIM4_IRQHandler(void)		//1ms
{
    if( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
    {     
				loop_cnt_10hz++;
				loop_cnt_20hz++;
			  loop_cnt_50hz++;
				loop_cnt_100hz++;
				loop_cnt_200hz++;
        TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);      
    }
}

void TIM4_Init(char clock,int Preiod)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
    
    TIM_DeInit(TIM4);

    TIM_TimeBaseStructure.TIM_Period = Preiod;
    TIM_TimeBaseStructure.TIM_Prescaler = clock-1;//1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4,TIM_FLAG_Update);

    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4,ENABLE);
    printf("Timer 4 init success...\r\n");
}	


void TimerNVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //TIM3
//    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
    //TIM4
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

} 


