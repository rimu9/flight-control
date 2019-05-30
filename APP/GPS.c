#include "GPS.h"

double Last_Longitude=0,Last_Latitude=0;
int32_t Longitude_Origion=0,Latitude_Origion=0;
double Longitude,Latitude;
double Longitude_Deg,Latitude_Deg,Longitude_Min,Latitude_Min;
float GPS_Ground_Speed=0;
float GPS_Yaw=0;
float GPS_Quality=0;
uint16_t GPS_Sate_Num=0;
float GPS_Speed_Resolve[2]={0,0};
u16 TimeBeijing[3];
char TimeStr[8];
Vector3_NED GPS_Vel={0};
float GPS_Pos_DOP=0;
uint8_t GPS_FixType=0;
uint8_t GPS_Fix_Flag[4]={0};
uint16_t Horizontal_Acc_Est=0;
uint16_t Vertical_Acc_Est=0;
uint16_t Speed_Acc_Est=0;
float High_GPS=0;


void GPS_PVT_Parse(void)
{
  Last_Longitude=Longitude;
  Last_Latitude=Latitude;
  //????
  TimeBeijing[0]=Ublox_Data[9]+8;//?
  TimeBeijing[1]=Ublox_Data[10];//?
  TimeBeijing[2]=Ublox_Data[11];//?

  GPS_FixType=Ublox_Data[21];//????

  GPS_Fix_Flag[0]=Ublox_Data[23]&0x01;//??????
  GPS_Fix_Flag[1]=(Ublox_Data[23]&0x02)>>1;//????????(DGPS)
  GPS_Fix_Flag[2]=(Ublox_Data[23]&0x3A)>>2;//???????
  GPS_Fix_Flag[3]=Ublox_Data[23]&0x20;//??????

  GPS_Sate_Num=Ublox_Data[24];//????????

  Longitude_Origion=Ublox_Data[25]//经度*10^7
             +(Ublox_Data[26]<<8)
              +(Ublox_Data[27]<<16)
               +(Ublox_Data[28]<<24);
  Longitude=Longitude_Origion*0.0000001f;//deg


  Latitude_Origion=Ublox_Data[29]//纬度*10^7
             +(Ublox_Data[30]<<8)
              +(Ublox_Data[31]<<16)
               +(Ublox_Data[32]<<24);
  Latitude=Latitude_Origion*0.0000001f;//deg


  Longitude_Deg=(int)(Longitude);//?????,??OLED??
  Longitude_Min=((int)((Longitude-Longitude_Deg)*10000000));
  Latitude_Deg=(int)(Latitude);
  Latitude_Min=((int)((Latitude-Latitude_Deg)*10000000));


  High_GPS=Ublox_Data[37]//GPS??????
             +(Ublox_Data[38]<<8)
              +(Ublox_Data[39]<<16)
               +(Ublox_Data[40]<<24);
  High_GPS/=1000;//m


  Horizontal_Acc_Est=Ublox_Data[41]//水平位置估计精度
             +(Ublox_Data[42]<<8)
              +(Ublox_Data[43]<<16)
               +(Ublox_Data[44]<<24);;
  Horizontal_Acc_Est*=0.01;//m


  Vertical_Acc_Est=Ublox_Data[45]//????????
             +(Ublox_Data[46]<<8)
              +(Ublox_Data[47]<<16)
               +(Ublox_Data[48]<<24);;
  Vertical_Acc_Est*=0.01;//m


  GPS_Vel.N=Ublox_Data[49]//GPS???????????  NED??
             +(Ublox_Data[50]<<8)
              +(Ublox_Data[51]<<16)
               +(Ublox_Data[52]<<24);
  GPS_Vel.N/=10;//cm/s  N


  GPS_Vel.E=Ublox_Data[53]//GPS????????????
             +(Ublox_Data[54]<<8)
              +(Ublox_Data[55]<<16)
               +(Ublox_Data[56]<<24);//mm/s
  GPS_Vel.E/=10;//cm/s  E


  GPS_Vel.D=Ublox_Data[57]//GPS??????? '?' ???
             +(Ublox_Data[58]<<8)
              +(Ublox_Data[59]<<16)
               +(Ublox_Data[60]<<24);
  GPS_Vel.D/=(-10);//cm/s  D

  GPS_Speed_Resolve[0]=GPS_Vel.N;//Y  Axis
  GPS_Speed_Resolve[1]=GPS_Vel.E;//X  Axis


  GPS_Ground_Speed=Ublox_Data[61]//??????
             +(Ublox_Data[62]<<8)
              +(Ublox_Data[63]<<16)
               +(Ublox_Data[64]<<24);
  GPS_Ground_Speed/=10;//cm/s


  GPS_Yaw=Ublox_Data[65]//运动航向角
             +(Ublox_Data[66]<<8)
              +(Ublox_Data[67]<<16)
               +(Ublox_Data[68]<<24);
  GPS_Yaw*=0.00001;//deg

  Speed_Acc_Est=Ublox_Data[69]//??????
             +(Ublox_Data[70]<<8)
              +(Ublox_Data[71]<<16)
               +(Ublox_Data[72]<<24);
  Speed_Acc_Est*=0.1;//cm/s


  GPS_Pos_DOP=Ublox_Data[77]
             +(Ublox_Data[78]<<8);
  GPS_Pos_DOP*=0.01;
  GPS_Quality=GPS_Pos_DOP;//??????
}
