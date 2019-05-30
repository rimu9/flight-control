#include "usart3.h"

void USART3_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
  GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;		
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate	= 100000;	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//?????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_Parity   = USART_Parity_Even;		
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);		
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	
	USART_Cmd(USART2, ENABLE);	
}

void USART3_IRQHandler(void)
{

  uint8_t current_byte;
  uint8_t clear; 
  static uint8_t byte_cnt = 0;	
  static uint8_t receive_flag = 0;					
	static uint8_t IDLE_flag = 0;	
  if(USART_GetITStatus(USART2,USART_IT_RXNE ) != RESET )
  {

    current_byte  = USART2->DR;							
		if(receive_flag == 0x66)	
		{
      Sbus_Buff[byte_cnt] = current_byte;	
			byte_cnt++;
			if(byte_cnt > 25)
			{
        Sbus_Buff[24] = 0x00;						//????
				byte_cnt = 0;
				receive_flag = 0;			
				IDLE_flag = 0;	
			}
		}
	else
	{
    if((IDLE_flag==1) && (current_byte==0x0F))
    {
      receive_flag  =0x66;
			 Sbus_Buff[0] = 0x0F ;
			 byte_cnt = 1;
    }
  }
	}
  if(USART_GetITStatus(USART2,UART_FLAG_IDLE) != RESET )
  {
    clear = USART2->SR;
		clear = USART2->DR;
		IDLE_flag = 1;
  }
}
