#ifndef _Tim_H_
#define _Tim_H_
#include "stm32f10x.h"
#include "USART1.h"

void LOOP_10HZ(void);
void LOOP_20HZ(void);
void LOOP_50HZ(void);
void LOOP_100HZ(void);
void LOOP_200HZ(void);
void loop(void);
void TIM4_Init(char clock,int Preiod);
//void TIM3_Init(char clock,int Preiod);
void TimerNVIC_Configuration(void);

extern volatile uint16_t anyCnt,anyCnt2,loop100HzCnt,loop200HzCnt;
extern uint8_t  loop200HzFlag,loop500HzFlag,loop50HzFlag,loop600HzFlag,loop100HzFlag,loop10HzFlag,loop20HzFlag;

#endif

