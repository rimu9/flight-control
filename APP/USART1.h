#ifndef __USART1_H_
#define __USART1_H_

#include "stm32f10x.h"
#include "public.h"

// USART Receiver buffer
#define RX_BUFFER_SIZE   128
#define TX_BUFFER_SIZE   128
extern unsigned char rx_buffer[RX_BUFFER_SIZE];
extern unsigned char tx_buffer[TX_BUFFER_SIZE];


typedef struct 
{
  uint16_t volatile Wd_Indx;
  uint16_t volatile Rd_Indx;
  uint16_t Mask;
  uint8_t *pbuf;
}UartBuf;

extern UartBuf UartTxbuf;
extern UartBuf UartRxbuf;


void UartBufClear(UartBuf *Ringbuf);

void UART1NVIC_Configuration(void);
void UART1_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
uint8_t Uart1_Put_Char(unsigned char DataToSend);
void USART1_Send(unsigned char *tx_buf, int len);
void USART1_SendData(unsigned char *Data, int16_t Datasize);

extern UartBuf UartTxbuf;
extern UartBuf UartRxbuf;
extern uint8_t UartBuf_RD(UartBuf *Ringbuf);
extern uint16_t UartBuf_Cnt(UartBuf *Ringbuf);
extern void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn);
void UartSendBuffer(uint8_t *dat, uint8_t len);

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void ANO_Data_Send_Status_USE_USB(void);
//test usart
void CommAppUpload(void);


#endif

