#ifndef __HMC5983_H
#define __HMC5983_H

#include "stm32f10x.h"
#include "IIC.h"
#include "delay.h"
#include "USART1.h"
#include "public.h"

#define HMC58X3_ADDR      0x3C // 7 bit address of the HMC58X3 used with the Wire library




#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0x00
#define HMC58X3_R_CONFB 0x01
#define HMC58X3_R_MODE 0x02
#define HMC58X3_R_XM 0x03
#define HMC58X3_R_XL 0x04

#define HMC58X3_R_YM 0x07  //!< Register address for YM.
#define HMC58X3_R_YL 0x08  //!< Register address for YL.
#define HMC58X3_R_ZM 0x05 //!< Register address for ZM.
#define HMC58X3_R_ZL 0x06  //!< Register address for ZL.

#define HMC58X3_R_STATUS 0x09
#define HMC58X3_R_IDA 0x0A
#define HMC58X3_R_IDB 0X0B
#define HMC58X3_R_IDC 0X0C

void HMC5883L_Init(void);
void HMC5883_Check(void);
void HMC58X3_getID(char id[3]);
void HMC58X3_Set(void);
void HMC58X3_FIFO_init(void);
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z);
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z);
void HMC58X3_writeReg(unsigned char reg, unsigned char val);
void HMC5983_ReadAll(struct Sense_Data *SensorTemp);

	
#endif

//------------------End of File----------------------------
