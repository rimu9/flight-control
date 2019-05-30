#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include "USART1.h"
#include "gpio.h"


#define SPI1_CS1_Disable 		GPIO_SetBits(GPIOB,GPIO_Pin_0)		//PB0		MPU6500
#define SPI1_CS1_Enable 	  GPIO_ResetBits(GPIOB,GPIO_Pin_0)

#define SPI1_CS2_Disable 	GPIO_SetBits(GPIOB,GPIO_Pin_2)		//PC5		MS5611
#define SPI1_CS2_Enable 	GPIO_ResetBits(GPIOB,GPIO_Pin_2)

#define SPI1_CS3_Disable	GPIO_SetBits(GPIOB,GPIO_Pin_1);			//PC4		HMC5983
#define SPI1_CS3_Enable		GPIO_ResetBits(GPIOB,GPIO_Pin_1)



void SPI1_Init(void);
u8 SPI1_Read_Write_Byte(uint8_t TxData);

#endif
