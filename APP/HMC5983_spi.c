#include "HMC5983_spi.h"


int16_t  HMC5883_spi_FIFO[3][11]; 


/**************************????********************************************
	   void  HMC58X3_spi_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新测量的ADC值
*******************************************************************************/
void  HMC58X3_spi_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_spi_FIFO[0][i-1]=HMC5883_spi_FIFO[0][i];
		HMC5883_spi_FIFO[1][i-1]=HMC5883_spi_FIFO[1][i];
		HMC5883_spi_FIFO[2][i-1]=HMC5883_spi_FIFO[2][i];
	}

	HMC5883_spi_FIFO[0][9]=x;
	HMC5883_spi_FIFO[1][9]=y;
	HMC5883_spi_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//??????????????
   		sum+=HMC5883_spi_FIFO[0][i];
	}
	HMC5883_spi_FIFO[0][10]=sum/10;	//??????

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_spi_FIFO[1][i];
	}
	HMC5883_spi_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_spi_FIFO[2][i];
	}
	HMC5883_spi_FIFO[2][10]=sum/10;
} //HMC58X3_newValues



/**************************????********************************************
	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
功能:	   通过FIFO数组更新最新磁力计数据
*******************************************************************************/
void HMC58X3_spi_getRaw(int16_t *x,int16_t *y,int16_t *z,struct Sense_Data *SensorTemp) {
   unsigned char vbuff[6];
//	 uint8_t i;
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
	 vbuff[0]=HMC5983_spi_Read_Reg(HMC5983_MagX_OutH);
	 vbuff[1]=HMC5983_spi_Read_Reg(HMC5983_MagX_OutL);
	 vbuff[2]=HMC5983_spi_Read_Reg(HMC5983_MagZ_OutH);
	 vbuff[3]=HMC5983_spi_Read_Reg(HMC5983_MagZ_OutL);
	 vbuff[4]=HMC5983_spi_Read_Reg(HMC5983_MagY_OutH);
	 vbuff[5]=HMC5983_spi_Read_Reg(HMC5983_MagY_OutL);
//   SPI1_CS3_Enable;
//   SPI1_Read_Write_Byte(HMC5983_MagX_OutH|0x80); 				//从加速度计的寄存器开始进行读取陀螺仪和加速度计的值//发送读命令+寄存器号
//	 for(i	=	0;i	< 6 ;i++)														//一共读取6字节的数据
//	 {
//		 vbuff[i]	=	SPI1_Read_Write_Byte(0xff);	//输入0xff,因为slave不识别
//		 delay_us(10);
//	 }	
	 SensorTemp->mag_origin_x=(float)(((int16_t)vbuff[0] << 8) | vbuff[1])/1090;
	 SensorTemp->mag_origin_y=(float)(((int16_t)vbuff[4] << 8) | vbuff[5])/1090;
	 SensorTemp->mag_origin_z=(float)(((int16_t)vbuff[2] << 8) | vbuff[3])/1090;
//	 SPI1_CS3_Disable;
   HMC58X3_spi_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_spi_FIFO[0][10];
   *y = HMC5883_spi_FIFO[1][10];
   *z = HMC5883_spi_FIFO[2][10];
}



/**************************????********************************************
原型:	   void HMC58X3_FIFO_init(void)
功能:	   上电连续读取50次地磁数据以初始化FIFO数组
*******************************************************************************/
void HMC58X3_spi_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_spi_getRaw(&temp[0],&temp[1],&temp[2],&senser_datas);
  delay_us(200); 
  }
}


/*
 * 函数名：HMC5983_Read_Reg
 * 描述  ：SPI读取寄存器
 * 输入  ：reg:指定的寄存器地址
 * 输出  ：reg_val：reg寄存器地址对应的值
 */ 
u8 HMC5983_spi_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI1_CS3_Enable;  										//使能SPI传输
	SPI1_Read_Write_Byte(reg | 0x80); 	//发送读命令+寄存器号
	reg_val = SPI1_Read_Write_Byte(0xff); //读取寄存器值
	SPI1_CS3_Disable;  									  //禁止
	return(reg_val);
}

/*
 * 函数名：HMC5983_Write_Reg
 * 描述  ：SPI写入寄存器
 * 输入  ：reg:指定的寄存器地址；value：写入的值
 * 输出  ：status：返回状态值
 */ 
u8 HMC5983_spi_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	SPI1_CS3_Enable;  										//使能SPI传输
	status = SPI1_Read_Write_Byte(reg); //发送写命令+寄存器号
	SPI1_Read_Write_Byte(value);				//写入寄存器值
	SPI1_CS3_Disable;   										//禁止
	return(status);											//返回状态值
}

//测试挂在SPI2上的HMC5983
void HMC5983_spi_check(void)
{
    char id[3];
	  id[0]=HMC5983_spi_Read_Reg(HMC5983_IdentRegA);
//	  delay_ms(10);
	  id[1]=HMC5983_spi_Read_Reg(HMC5983_IdentRegB);
//	  delay_ms(10);
	  id[2]=HMC5983_spi_Read_Reg(HMC5983_IdentRegC);
//	  delay_ms(10);
	  if((id[0]==0x48)&&(id[1]==0x34)&&(id[2]==0x33))
		{
			printf("HMC5883L check success...\r\n");
		}
		else
		{
			printf("HMC5883L not found...\r\n");
		}
}


//初始化HMC5933磁场测量范围
void HMC5983_spi_init(void)
{
	HMC5983_spi_check();
  HMC5983_spi_Write_Reg(HMC5983_ConfRegA,0xBC);
	HMC5983_spi_Write_Reg(HMC5983_ConfRegB,0x20);
	HMC5983_spi_Write_Reg(HMC5983_ModeReg,0x00);
	HMC58X3_spi_FIFO_init();
	printf("HMC5883L set completed...\r\n");
}



//读取全部6Bytes磁场数据
//int16_t *xdata	: X/Y/Z的磁场数据，IC寄存器里的是X、Z、Y，已经转换为了X、Y、Z
//float *Gauss		:归一化后的磁场数据，单位是高斯G	1T=10000 Gauss
//函数获取测量值耗时66us
void HMC5983_spi_Read_All(struct Sense_Data *SensorTemp)
{
	int16_t x,y,z;
	HMC58X3_spi_getRaw(&(x),&(y),&(z),&senser_datas);
	SensorTemp->mag_filtered_x=(float)x-22;
	SensorTemp->mag_filtered_y=(float)y+139;
	SensorTemp->mag_filtered_z=(float)+z;
	
}
