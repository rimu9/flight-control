#include "MS5611.h"

uint16_t Cal_C[7];
float readdata;
float dT,T1,T2,Aux,OFF2,SENS2;
double OFF,SENS;  


void MS5611_Reset(void) 
{
	IIC_Start();
    IIC_Send_Byte(MS5611_ADDR); 
	IIC_Wait_Ack();
    IIC_Send_Byte(MS561101BA_RST);
	IIC_Wait_Ack();	
    IIC_Stop();
}

/**************************????********************************************
原型:		void MS561101BA_readPROM(void)
功能:	   读取ms5611工厂标定值用于修正温度和气压读数
*******************************************************************************/
void MS5611_ReadPROM(void) 
{
	u8  inth,intl,d[12];
	int i;
	for (i=0;i<=6;i++) 
	{
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR);
			IIC_Wait_Ack();
			IIC_Send_Byte(MS5611_PROM_Setup + (i * 2));
			IIC_Wait_Ack();	
			IIC_Stop();
			delay_us(5);
			IIC_Start();
			IIC_Send_Byte(MS5611_ADDR+1);  //??????	
			delay_us(1);
			IIC_Wait_Ack();
			inth = IIC_Read_Byte(1);  //?ACK????
			delay_us(1);
			intl = IIC_Read_Byte(0);	 //??????NACK
			IIC_Stop();
			
		d[i*2]=inth;  d[i*2+1]=intl;
			Cal_C[i] = (((uint16_t)inth << 8) | intl);
	}
//	USART1_SendData((unsigned char *)(d),sizeof(d));
}

void MS5611_Init(void)
{
	MS5611_Reset();
	delay_ms(100);
	MS5611_ReadPROM();
	delay_ms(100);
} 



void MS5611_CONVERSION(uint8_t command)
{
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //???
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //?????
	IIC_Wait_Ack();	
	IIC_Stop();
}

uint32_t MS5611_GetTemp(uint8_t command)
{
	uint32_t conversion=0;
	uint8_t temp[3]; 	//conv1,conv2,conv3
	
	MS5611_CONVERSION(command);
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //???
	IIC_Wait_Ack();
	IIC_Send_Byte(0);// start read sequence
	IIC_Wait_Ack();	
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR+1);  //??????	
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //?ACK????  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //?ACK????  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //?NACK???? bit 0-7
	IIC_Stop();

	conversion= ((uint32_t)temp[0]<<16) + ((uint32_t)temp[1]<<8) + temp[2];

	return conversion;
	
}

uint32_t MS5611_GetPress(uint8_t command)
{
	uint32_t conversion=0;
	uint8_t temp[3]; 	//conv1,conv2,conv3
	
	MS5611_CONVERSION(command);
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR); //???
	IIC_Wait_Ack();
	IIC_Send_Byte(0);// start read sequence
	IIC_Wait_Ack();	
	IIC_Stop();
	
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR+1);  //??????	
	IIC_Wait_Ack();
	temp[0] = IIC_Read_Byte(1);  //?ACK????  bit 23-16
	temp[1] = IIC_Read_Byte(1);  //?ACK????  bit 8-15
	temp[2] = IIC_Read_Byte(0);  //?NACK???? bit 0-7
	IIC_Stop();

	conversion= ((uint32_t)temp[0]<<16) + ((uint32_t)temp[1]<<8) + temp[2];

	return conversion;
	
}




//读取数字温度
void MS5611_GetTemperature(struct Sense_Data *ms5611data)
{
	readdata= MS5611_GetTemp(MS561101BA_D2_OSR_4096);	

	dT=readdata - (((uint32_t)Cal_C[5])<<8);
	T1=(int16_t)(2000+dT*((uint32_t)Cal_C[6])/8388608);
	
	ms5611data->temperature=T1;
	
}
//读取数字气压
void MS5611_GetPressure(struct Sense_Data *ms5611data)
{
	readdata= MS5611_GetPress(MS561101BA_D1_OSR_4096);
	ms5611data->pressure_origin=readdata;
}



void MS561101BA_getPressure(void) 
{
	MS5611_GetTemperature(&senser_datas);
	MS5611_GetPressure(&senser_datas);
	
	OFF=(uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT)/128;
	SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256;
	//温度补偿
	if(T1 < 2000)// second order temperature compensation when under 20 degrees C
	{
		T2 = (dT*dT) / 0x80000000;
		Aux = (T1-2000)*(T1-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(T1 < -1500)
		{
			Aux = (T1+1500)*(T1+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
		}
	}
	else  //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}	
	T1 -= T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	
	
	senser_datas.pressure_filtered=(((((int64_t)senser_datas.pressure_origin) * SENS)/2097152) -OFF) / 32768;
}



float MS5611_GetAlt(void)
{
	//气压对绝对高度的转换，输入Pa，输出m	 
  float A = senser_datas.pressure_filtered/101325;
  float B = 1/5.25588;
  float C =pow(A,B);  //求x的y次方
  C = 1 - C;
  C = C /0.0000225577;
	
	return C; 
}

void MS5611_Get_Altittude()
{
   float Alt;
	MS561101BA_getPressure();
	Alt= MS5611_GetAlt()-senser_datas.altitude_start_quiet;
	senser_datas.altitude_orign=Alt;
}

	

