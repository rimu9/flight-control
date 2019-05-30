#include "SYS_FUN.h"
#include "stm32f10x.h"
#include "Tim.h"
#include "USART1.h"
#include "usart2.h"
#include "U_100.h"


char SysClock;  


void NVIC_INIT(void)
{
    TimerNVIC_Configuration();//��ʱ���ж� 0 0
	  EXTI_NVIC_Configuration();//�������ж� 0 2
	  USART2NVIC_Configuration();//GPS�ô������ȼ� 1 1
    UART1NVIC_Configuration();//����λ������1�ж�  2 0
}

//����ʱ�ӼĴ�����λ
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000;
    RCC->APB2RSTR = 0x00000000;

    RCC->AHBENR = 0x00000014;  
    RCC->APB2ENR = 0x00000000; 
    RCC->APB1ENR = 0x00000000;
    RCC->CR |= 0x00000001;     
    RCC->CFGR &= 0xF8FF0000;   
    RCC->CR &= 0xFEF6FFFF;    
    RCC->CR &= 0xFFFBFFFF;     
    RCC->CFGR &= 0xFF80FFFF;   
    RCC->CIR = 0x00000000;     
}



/********************************************
    ʹ���ڲ�DCO����ϵͳʱ��      
********************************************/
char SystemClock_HSI(u8 PLL)
{
    RCC->CR|=1<<0;              
    RCC->CR|=0<<16;              
    RCC->CR|=1<<18;
    while(!((RCC->CR)&(1<<1))); 
    RCC->CFGR|=(PLL-2)<<18;     
    RCC->CFGR|=0<<16;           
    RCC->CR|=1<<24;             
    while(!((RCC->CR)&(1<<25)));
    RCC->CFGR|=2<<0;            
    SysClock=4*PLL;             
    return SysClock;
}


/********************************************
  ʹ���ⲿ������ʱ��Դ
********************************************/
char SystemClock_HSE(u8 PLL)
{
    unsigned char temp=0;
    MYRCC_DeInit();		    
    RCC->CR|=1<<16;       
    while(!(RCC->CR>>17));
    RCC->CFGR=0X00000400; 
    PLL-=2;//??2???
    RCC->CFGR|=PLL<<18;  
    RCC->CFGR|=1<<16;	    
    FLASH->ACR|=0x32;	    
    RCC->CR|=0x01000000; 
    while(!(RCC->CR>>25));
    RCC->CFGR|=0x00000002;
    while(temp!=0x02)    
    {
        temp=RCC->CFGR>>2;
        temp&=0x03;
    }

    SysClock=(PLL+2)*8;
    return SysClock;
}
