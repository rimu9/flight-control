#include "HMC5983.h"


int16_t  HMC5883_FIFO[3][11]; 




/**************************????********************************************
	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
输入:    reg  寄存器地址
			  val   写入的值	
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val) {
  IICwriteByte(HMC58X3_ADDR,reg,val);
}



/**************************????********************************************
	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
功能:	  均值滤波，更新FIFO数组
输入:  最新测量的ADC值
*******************************************************************************/
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<10;i++){
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][9]=x;
	HMC5883_FIFO[1][9]=y;
	HMC5883_FIFO[2][9]=z;

	sum=0;
	for(i=0;i<10;i++){	//??????????????
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;	//??????

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
} //HMC58X3_newValues


/**************************????********************************************
	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
功能:	   通过FIFO数组更新最新磁力计数据
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z) {
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   IICreadBytes(HMC58X3_ADDR,HMC58X3_R_XM,6,vbuff);
   HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1],((int16_t)vbuff[4] << 8) | vbuff[5],((int16_t)vbuff[2] << 8) | vbuff[3]);
   *x = HMC5883_FIFO[0][10];
   *y = HMC5883_FIFO[1][10];
   *z = HMC5883_FIFO[2][10];
}

/*将数据读入全局结构体变量中方便调用，该函数作为接口函数供imu单元调用*/
void HMC5983_ReadAll(struct Sense_Data *SensorTemp)
{
	int16_t x,y,z;
	HMC58X3_getRaw(&(x),&(y),&(z));
	SensorTemp->mag_filtered_x=(float)x/1090;
	SensorTemp->mag_filtered_y=(float)x/1090;
	SensorTemp->mag_filtered_z=(float)x/1090;
}


/**************************????********************************************
原型:	   void HMC58X3_FIFO_init(void)
功能:	   上电连续读取50次地磁数据以初始化FIFO数组
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
  int16_t temp[3];
  unsigned char i;
  for(i=0;i<50;i++){
  HMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  delay_us(200); 
  }
}


/**************************????********************************************
   void HMC58X3_init(u8 setmode)
 //设置hmc5983工作模式
*******************************************************************************/
void HMC58X3_Set() {

  HMC58X3_writeReg(HMC58X3_R_CONFA, 0xF8); //  1-enable temperature,  11-8 samples averaged, 110-75Hz frequency, 00-no artificial bias.
  HMC58X3_writeReg(HMC58X3_R_CONFB, 0x20);  //+- 1.3G      1090 Gain(LSb/Gauss)
  HMC58X3_writeReg(HMC58X3_R_MODE,  0x00);  //Continuous-Measurement Mode

}


/**************************????********************************************
	  void HMC58X3_getID(char id[3])
// 读取芯片ID
*******************************************************************************/
void HMC58X3_getID(char id[3]) 
{
      id[0]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDA);  
      id[1]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDB);
      id[2]=I2C_ReadOneByte(HMC58X3_ADDR,HMC58X3_R_IDC);
	printf("%c",id[0]);
	printf("%c",id[1]);
	printf("%c",id[2]);
}   

/**************************????********************************************
  void HMC5883_Check()
//检测hmc5983模块存在性
*******************************************************************************/
void HMC5883_Check() 
{
  char ID_temp[3];
  HMC58X3_getID(ID_temp);
  
  if((ID_temp[0]==0x48)&&(ID_temp[1]==0x34)&&(ID_temp[2]==0x33))//HMC5983 ID固定返回三个字节0x48 0x34 0x33
  printf("HMC5883L check success...\r\n");
  else printf("HMC5883L not found...\r\n");
}   


/**************************????********************************************
   //初始化HMC5983模块
*******************************************************************************/
void HMC5883L_Init(void)
{ 
  HMC5883_Check();
  HMC58X3_Set(); 
  HMC58X3_FIFO_init();
}


