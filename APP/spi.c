#include "spi.h"


void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);	//GPIOA时钟使能 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE );												//SPI1时钟使能 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	My_GPIO_Init();
	
	SPI1_CS3_Disable;
  SPI1_CS2_Disable;
  SPI1_CS1_Disable;	//片选取消
	GPIO_SetBits(GPIOA,GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);  									//PB13/14/15上拉
	
	SPI_Cmd(SPI1,DISABLE);  																										//SPI1失能
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  				//SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;																//SPI主机
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;														//发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																	//时钟悬空低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;																//数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;																		//NSS信号由软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;					//定义波特率预分频的值:波特率预分频值为16（72M/8=9M）
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;													//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;																		//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure); 																			  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	SPI_Cmd(SPI1,ENABLE);																												//使能SPI1
	SPI1_Read_Write_Byte(0xff);		
}      

/*
 * 函数名：SPI1_Read_Write_Byte
 * 描述  ：读写一个字节
 * 输入  ：TxData:要写入的字节
 * 输出  ：读取到的字节
 */ 
u8 SPI1_Read_Write_Byte(uint8_t TxData)
{		
	u8 retry = 0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) 	//检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry > 250)	return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); 																//通过外设SPIx发送一个数据
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry > 250) return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); 															//返回通过SPIx最近接收的数据					    
}




