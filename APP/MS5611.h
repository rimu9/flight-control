#ifndef __MS5611_H
#define __MS5611_H

#include "IIC.h"
#include "delay.h"
#include "public.h"
#include "USART1.h"
#include "math.h"
// addresses of the device CSB =0 
#define MS5611_ADDR         0xEE     // default I2C address

#define  MS561101BA_RST 		0x1E 
#define  MS561101BA_D1_OSR_4096 0x48 				//×ª»»ÆøÑ¹ÃüÁî
#define  MS561101BA_D2_OSR_4096 0x58 				//×ª»»ÎÂ¶ÈÃüÁî
#define  MS561101BA_ADC_RD 		0x00 
#define  MS5611_PROM_Setup 	0xA0
#define  MS5611_PROM_C1 		0xA2
#define  MS5611_PROM_C2 		0xA4
#define	 MS5611_PROM_C3 		0xA6
#define	 MS5611_PROM_C4 		0xA8
#define	 MS5611_PROM_C5 		0xAA
#define	 MS5611_PROM_C6 		0xAC
#define	 MS5611_PROM_CRC 		0xAE


void MS5611_Reset(void);
void MS5611_ReadPROM(void);
void MS5611_Init(void);
void MS5611_CONVERSION(uint8_t command);
uint32_t MS5611_GetTemp(uint8_t command);
uint32_t MS5611_GetPress(uint8_t command);
void MS5611_GetTemperature(struct Sense_Data *ms5611data);
void MS561101BA_getPressure(void);
float MS5611_GetAlt(void);
void MS5611_Get_Altittude(void);




#endif
