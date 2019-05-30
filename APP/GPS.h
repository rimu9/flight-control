#ifndef __GPS_H
#define	__GPS_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "USART1.h"
#include <math.h>
#include "usart2.h"
#include "public.h"

#define EARTHR 6371004
//-----GNSS fixType-----//
typedef enum
{
  NO_FIX = 0x00,//???
  DEAD_RECKONING_ONLY = 0x01,//?????
  FIX_2D = 0x02,//2???
  FIX_3D = 0x03,//3???
  GNSS_DEAD_RECKONING = 0x04,//GPS?????
  TIME_ONLY_FIX = 0x05//??????
}GNSS_FixType_Def;


void GPS_PVT_Parse(void);

extern double Last_Longitude,Last_Latitude;
extern int32_t Longitude_Origion,Latitude_Origion;
extern double Longitude,Latitude;

extern double Longitude_Deg,Latitude_Deg;
extern double Longitude_Min,Latitude_Min;
extern u16 TimeBeijing[3];
extern char TimeStr[8];
extern float GPS_Quality;
extern uint16_t GPS_Sate_Num;
extern float GPS_Yaw;
extern float GPS_Ground_Speed;
extern float GPS_Speed_Resolve[2];
extern Vector3_NED GPS_Vel;
extern float GPS_Pos_DOP;
extern uint8_t  GPS_Fix_Flag[4];

extern uint8_t GPS_FixType;
extern float High_GPS;

extern uint16_t Horizontal_Acc_Est;

#endif
