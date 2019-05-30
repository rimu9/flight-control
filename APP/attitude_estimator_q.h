#ifndef _ATTITUDE_ESTIMATOR_Q_H_
#define _ATTITUDE_ESTIMATOR_Q_H_
#include "stm32f10x.h"
#include "public.h"
#include "USART1.h"
#include "usart2.h"
#include "GPS.h"
#include "math.h"

#define gyro_bias_min -0.1
#define gyro_bias_max  0.1


float invSqrt(float number);
void q_update(Vector3f *temp);
void Yaw_to_quaternion(Quaternion *Quaternion_TEMP ,float fi ,float theta ,float gama); //欧拉角转化为四元素
Quaternion q1_q2(Quaternion *q1 ,Quaternion *q2);
Vector3f NED_TO_BODY(float x ,float y ,float z);
Vector3f BODY_TO_NED(float x ,float y ,float z);
float wrap_50per_pi(float data);
void q_normalized(void);
float math_constrain(float data ,float min ,float max);
uint8_t update(float dt);
void attitude_estimator_q_main(void);
uint8_t is_finite(float min ,float max);
void _init(void);

#endif

