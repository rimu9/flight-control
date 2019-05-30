#ifndef _USART2_H
#define _USART2_H

#include "stm32f10x.h"

void UART2_Send(unsigned char tx_buf);
void USART2_Init(unsigned long bound);
void USART2NVIC_Configuration(void);



extern uint8_t Ublox_Data[95];



#endif
