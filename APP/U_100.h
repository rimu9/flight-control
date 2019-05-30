#ifndef _U_100_H_
#define _U_100_H_

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "USART1.h"
#include "gpio.h"
#include "delay.h"



#define US_100_Distance_CMD    0x55
#define US_100_Temperature_CMD 0x50

float US_100_Distance(uint8_t MSB,uint8_t LSB);
float US_100_Temperature(uint8_t data);
extern float US_Distance;
extern uint16_t HC_SR04_RSSI;

#define HC_SR04_OUT_LOW    GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#define HC_SR04_OUT_HIGH   GPIO_SetBits(GPIOA,GPIO_Pin_8);

void HC_SR04_Init(void);
void EXTI_NVIC_Configuration(void);
void HC_SR04_Start(void);
extern float HC_SR04_Distance;
extern uint16_t Exti_Cnt;

#endif


