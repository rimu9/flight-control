#include "U_100.h"


float US_Distance=0;


float US_100_Distance(uint8_t MSB,uint8_t LSB)
{
  return (256*(MSB)+(LSB))/1.0;
}

float US_100_Temperature(uint8_t data)
{
  return (data-45)/1.0;
}




uint8_t HC_SR04_StartFlag=0;
float HC_SR04_Distance=0;
uint16_t HC_SR04_RSSI=1;

void HC_SR04_Start(void)
{
  if(HC_SR04_StartFlag==0)
  {
    HC_SR04_OUT_HIGH;
    delay_us(20);
    HC_SR04_OUT_LOW;
    EXTI->IMR |=EXTI_Line1;
    HC_SR04_StartFlag=1;
    HC_SR04_RSSI=255;
  }
   HC_SR04_RSSI--;
}

void HC_SR04_Init(void)
{
      GPIO_InitTypeDef  GPIO_InitStructure;
      EXTI_InitTypeDef EXTI_InitStructure;

      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
      GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//????
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//????
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
      EXTI_InitStructure.EXTI_Line = EXTI_Line1;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
      EXTI_Init(&EXTI_InitStructure);

      //EXTI->IMR &=~EXTI_Line1;//??????
}

void EXTI_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void HC_SR04_UP()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


void HC_SR04_DN()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

uint32_t Test_Cnt1,Test_Cnt2=0,Test_Delta=0;
uint16_t Exti_Cnt=0;
uint16_t Sample_Cnt=0;
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
   Sample_Cnt++;
   if(Sample_Cnt==1)//????
   {
    Exti_Cnt++;
    Test_Cnt1=micros();
    HC_SR04_DN();
   }
   else//????
   {
   Test_Cnt2=micros();
   HC_SR04_StartFlag=0;
   HC_SR04_UP();
   //EXTI->IMR &=~EXTI_Line1;//??????
   Sample_Cnt=0;
   Test_Delta=(Test_Cnt2-Test_Cnt1);
   HC_SR04_Distance=(Test_Cnt2-Test_Cnt1)*(340)/2000; //µ¥Î»mm
		 
		 USART1_SendData((unsigned char *)(&HC_SR04_Distance),sizeof(HC_SR04_Distance));

   }

  }
  EXTI_ClearITPendingBit(EXTI_Line1);
}
/***************************????,????,???????:?????:1094744141@qq.com*********************************/

