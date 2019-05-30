#ifndef __MS5611_SPI_H
#define __MS5611_SPI_H

#include "spi.h"
#include "delay.h"
#include "public.h"
#include "USART1.h"
#include "gpio.h"
#include "math.h"


#define  MS5611_RA_RST 		0x1E 
#define  MS5611_RA_D1_OSR_4096 0x48 				//×ª»»ÆøÑ¹ÃüÁî
#define  MS5611_RA_D2_OSR_4096 0x58 				//×ª»»ÎÂ¶ÈÃüÁî
#define  MS5611_RA_ADC_RD 		0x00 
#define  MS5611_RA_PROM_Setup 	0xA0
#define  MS5611_RA_PROM_C1 		0xA2
#define  MS5611_RA_PROM_C2 		0xA4
#define	 MS5611_RA_PROM_C3 		0xA6
#define	 MS5611_RA_PROM_C4 		0xA8
#define	 MS5611_RA_PROM_C5 		0xAA
#define	 MS5611_RA_PROM_C6 		0xAC
#define	 MS5611_RA_PROM_CRC 		0xAE





float MS5611_Command(uint8_t command);
void MS5611_spi_Read(uint8_t reg,uint8_t *p);
void MS5611_SPI_ReadPROM(void);
void MS5611_SPI_Reset(void);
void MS5611_SPI_Init(void);
void MS5611_SPI_Get_Altittude(void);
float MS5611_SPI_GetAlt(void);
void MS5611_SPI_GetPressure(void);
void MS5611_SPI_GetTemperature(struct Sense_Data *ms5611data);


#endif
