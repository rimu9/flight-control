#include "usart2.h"
#include "USART1.h"




void UART2_Send(unsigned char tx_buf)
{
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);//?????fputc?????
  USART_SendData(USART2 , tx_buf);//????????????
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
}


void USART2_Init(unsigned long bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO
                               |RCC_APB2Periph_GPIOA , ENABLE);//??2

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//??2 ??

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = bound;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;//no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//enable tx and rx
	USART_Init(USART2, &USART_InitStructure);//

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//rx interrupt is enable
	USART_Cmd(USART2, ENABLE);


        //USART2_Send((unsigned char *)Buffer,2);
        UART2_Send(0xAA);
}


void USART2NVIC_Configuration()
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}








uint8_t ucRxBuffer[9];
int ucRxCnt=0;	
int makesure=0,start=0,try_cnt=0;




u8 GPS_Buf[2][100]={0};
unsigned char GPS_GPGGA_Buf[6];
unsigned int GPS_Data_Cnt=0;
unsigned GPSx_Cnt=0,GPSx_Finish_Flag=1,GPSx_Data_Cnt=0;
u16 GPS_ISR_CNT=0;
uint8_t GPxCnt=0;

uint16_t Ublox_Try_Cnt=0;
uint8_t Ublox_Try_Buf[5]={0};
uint16_t Ublox_Try_Makesure=0;
uint16_t Ublox_Try_Start=0;

uint8_t Ublox_Data[95]={0};
uint16_t Ublox_Cnt=0;
uint16_t Ublox_Receive=0;
uint16_t GPS_Update_finished=0;
uint16_t GPS_Update_finished_Correct_Flag=0;

//Testime GPS_Time_Delta;
void USART2_IRQHandler(void)
{
 unsigned char ch;
if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
{
	ch=USART_ReceiveData(USART2);
/***********************************************************************************************
***************************GPSÓÃ´úÂë************************************************************
*********************************************************************************************/
	if(Ublox_Try_Makesure==1)
  {
     Ublox_Data[Ublox_Cnt++]=ch;
     if(Ublox_Cnt==94)
     {
       Ublox_Cnt=0;
       Ublox_Try_Makesure=0;
       GPS_Update_finished=1;
       GPS_Update_finished_Correct_Flag=1;
       //Test_Period(&GPS_Time_Delta);//GPS????????
     }
  }

  if(Ublox_Try_Makesure==0
     &&ch==0xB5)
  {
    Ublox_Try_Start=1;
    Ublox_Try_Cnt=0;
  }

  if(Ublox_Try_Start==1)
  {
    Ublox_Try_Cnt++;
    if(Ublox_Try_Cnt>=5)
    {
      Ublox_Try_Start=0;
      Ublox_Try_Cnt=0;

      if(ch==0x5C) Ublox_Try_Makesure=1;
      else Ublox_Try_Makesure=0;
    }
  }	
}
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);

}

