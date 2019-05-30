#ifndef __HMC5983_H
#define __HMC5983_H			    
#include "stm32f10x_it.h"
#include "spi.h"
#include "gpio.h"
#include "public.h"
#include "delay.h"
#include "USART1.h"

#define HMC5983_ConfRegA	0x00
#define HMC5983_ConfRegB	0x01
#define HMC5983_ModeReg		0x02

#define HMC5983_MagX_OutH	0x03
#define HMC5983_MagX_OutL	0x04
#define HMC5983_MagZ_OutH	0x05
#define HMC5983_MagZ_OutL	0x06
#define HMC5983_MagY_OutH	0x07
#define HMC5983_MagY_OutL	0x08

#define HMC5983_StatusReg		0x09
#define HMC5983_IdentRegA		0x0A
#define HMC5983_IdentRegB		0x0B
#define HMC5983_IdentRegC		0x0C
#define HMC5983_TemperatureH		0x31
#define HMC5983_TemperatureL		0x32


void  HMC58X3_spi_newValues(int16_t x,int16_t y,int16_t z);
void HMC58X3_spi_getRaw(int16_t *x,int16_t *y,int16_t *z,struct Sense_Data *SensorTemp);
void HMC58X3_spi_FIFO_init(void);
u8 HMC5983_spi_Read_Reg(uint8_t reg);
u8 HMC5983_spi_Write_Reg(uint8_t reg,uint8_t value);
void HMC5983_spi_check(void);
void HMC5983_spi_init(void);
void HMC5983_spi_Read_All(struct Sense_Data *SensorTemp);

#endif
