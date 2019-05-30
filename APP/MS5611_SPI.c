#include "MS5611_SPI.h"


uint16_t PROM_Data[8];
float dT_SPI,T1_SPI,T2_SPI,Aux_SPI,OFF2_SPI,SENS2_SPI;
double OFF_SPI,SENS_SPI;  



/*
   读取ms5611 PROM出厂数据用函数，返回两字节
*/
void MS5611_spi_Read(uint8_t reg,uint8_t *p)
{
	SPI1_CS2_Enable;  										//使能SPI传输
	SPI1_Read_Write_Byte(reg|0x80); 	//发送读命令+寄存器号
	SPI1_CS2_Disable;
	delay_ms(10);
	SPI1_CS2_Enable;
	p[0] = SPI1_Read_Write_Byte(MS5611_RA_ADC_RD); //读取寄存器值
	p[1]=SPI1_Read_Write_Byte(MS5611_RA_ADC_RD);
	SPI1_CS2_Disable;  									  //禁止
}
/*
   读取ms5611PROM里出厂设置数据
*/
void MS5611_SPI_ReadPROM(void)
{
	uint8_t val[16]={0};
	int i;
	MS5611_spi_Read(MS5611_RA_PROM_Setup,&val[0]);
	MS5611_spi_Read(MS5611_RA_PROM_C1,&val[2]);
	MS5611_spi_Read(MS5611_RA_PROM_C2,&val[4]);
	MS5611_spi_Read(MS5611_RA_PROM_C3,&val[6]);
	MS5611_spi_Read(MS5611_RA_PROM_C4,&val[8]);
	MS5611_spi_Read(MS5611_RA_PROM_C5,&val[10]);
  MS5611_spi_Read(MS5611_RA_PROM_C6,&val[12]);
	MS5611_spi_Read(MS5611_RA_PROM_CRC,&val[12]);
	for(i=0;i<8;i++)
	{
		PROM_Data[i]= (((uint16_t)val[i*2]<<8)|val[i*2+1]);
	}
//  printf("ms5611 read prom success...");
//	USART1_SendData((unsigned char *)(&val),sizeof(val));
}
/*
   ms5611重置
*/
void MS5611_SPI_Reset(void)
{
	SPI1_CS2_Enable;
	SPI1_Read_Write_Byte(MS5611_RA_RST);
	SPI1_CS2_Disable;
}

/*
   气压计初始化函数
*/
void MS5611_SPI_Init(void)
{
	MS5611_SPI_Reset();
	delay_ms(100);
	MS5611_SPI_ReadPROM();
	delay_ms(100);
	printf("ms5611 init success...");
}



//指令：转换
float MS5611_Command(uint8_t command)
{
	uint8_t conv[3]={0};
	float conversion;
	
	SPI1_CS2_Enable;
	SPI1_Read_Write_Byte(command|0x80); 	//发送读命令+寄存器号
	SPI1_CS2_Disable;
	delay_ms(10);
	SPI1_CS2_Enable;
	conv[0] = SPI1_Read_Write_Byte(MS5611_RA_ADC_RD); //读取寄存器值
	conv[1]=SPI1_Read_Write_Byte(MS5611_RA_ADC_RD);
	conv[2]=SPI1_Read_Write_Byte(MS5611_RA_ADC_RD);
	conversion= ((uint32_t)conv[0]<<16) + ((uint32_t)conv[1]<<8) + conv[2];
	SPI1_CS2_Disable;
	return conversion;
}


//读取温度数据以进行温度补偿
void MS5611_SPI_GetTemperature(struct Sense_Data *ms5611data)
{
	float data;
	data=MS5611_Command(MS5611_RA_D2_OSR_4096);
	dT_SPI=data - (((uint32_t)PROM_Data[5])<<8);
	T1_SPI=(int16_t)(2000+dT_SPI*((uint32_t)PROM_Data[5])/8388608);
	ms5611data->temperature=T1_SPI;
//	USART1_SendData((unsigned char *)(&T1_SPI),sizeof(T1_SPI));

}
//读取原始气压数据，即未进行温度补偿前的数据
void MS5611_SPI_Get_Origin_Pressure(struct Sense_Data *ms5611data)
{
	float data;
  data=MS5611_Command(MS5611_RA_D1_OSR_4096);
	ms5611data->pressure_origin=data;
	
//	USART1_SendData((unsigned char *)(&data),sizeof(data));
}
//将读取的温度，气压数据，进行温度补偿得到真实气压数据
void MS5611_SPI_GetPressure(void)
{
	MS5611_SPI_GetTemperature(&senser_datas);
	MS5611_SPI_Get_Origin_Pressure(&senser_datas);
	
	OFF_SPI=(double)(PROM_Data[2]<<16)+((double)PROM_Data[4]*dT_SPI)/128;
	SENS_SPI=(double)(PROM_Data[1]<<15)+((double)PROM_Data[3]*dT_SPI)/256;
	//温度补偿
	if(T1_SPI < 2000)// second order temperature compensation when under 20 degrees C
	{
		T2_SPI = (dT_SPI*dT_SPI) / 0x80000000;
		Aux_SPI = (T1_SPI-2000)*(T1_SPI-2000);
		OFF2_SPI = 2.5*Aux_SPI;
		SENS2_SPI = 1.25*Aux_SPI;
		if(T1_SPI < -1500)
		{
			Aux_SPI = (T1_SPI+1500)*(T1_SPI+1500);
			OFF2_SPI = OFF2_SPI + 7*Aux_SPI;
			SENS2_SPI = SENS_SPI + 5.5*Aux_SPI;
		}
	}
	else  //(Temperature > 2000)
	{
		T2_SPI = 0;
		OFF2_SPI = 0;
		SENS2_SPI = 0;
	}	
	T1_SPI -= T2_SPI;
	OFF_SPI = OFF_SPI - OFF2_SPI;
	SENS_SPI = SENS_SPI - SENS2_SPI;	
	
	senser_datas.pressure_filtered=(((((float)senser_datas.pressure_origin) * SENS_SPI)/2097152) -OFF_SPI) / 32768;
}
//将气压数据转换为高度数据
float MS5611_SPI_GetAlt(void)
{
		//气压对绝对高度的转换，输入Pa，输出m	 
  float A = senser_datas.pressure_filtered/101325;
  float B = 1/5.25588;
  float C =pow(A,B);  //求x的y次方
  C = 1 - C;
  C = C*44300;
	
	return C; 
}
//整合读取高度数据过程，为imu调用的函数
void MS5611_SPI_Get_Altittude(void)
{
	float Alt;
	MS5611_SPI_GetPressure();
	Alt= MS5611_SPI_GetAlt()-senser_datas.altitude_start_quiet;
	senser_datas.altitude_orign=Alt;
}

