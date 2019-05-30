#include "attitude_estimator_q.h"



extern Quaternion q,q_d;
float mag_decl=0;
float vel_dt;
float vel_prev_t,dt,att_last_time,att_now_time;
Vector3_NED vel_prev;
Vector3f pos_acc , gyro_bias ,_rates;
Vector3f zero ={0,0,0};
float _w_mag=30,_w_accl=30,_w_gyro_bias=15;

//函数名:invSqrt(void)
//求平方根的倒数
//Carmack求平方根算法,使用魔数0x5f375a86
float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void q_update(Vector3f *temp)
{
	Quaternion q_ed;
	q_ed.q0=q.q0; q_ed.q1=q.q1; q_ed.q2=q.q2; q_ed.q3=q.q3;
	q.q0=q_ed.q0 - 0.5f*dt*q_ed.q1*temp->x -0.5f*dt*q_ed.q2*temp->y -0.5f*dt*q_ed.q3*temp->z;
	q.q1=q_ed.q0*0.5f*dt*temp->x + q_ed.q1 +0.5f*dt*q_ed.q2*temp->z -0.5f*dt*q_ed.q3*temp->y;
	q.q2=q_ed.q0*0.5f*dt*temp->y - 0.5f*dt*q_ed.q1*temp->z +q_ed.q2 +0.5f*dt*q_ed.q3*temp->x;
	q.q3=q_ed.q0*0.5f*dt*temp->z + 0.5f*dt*q_ed.q1*temp->y -0.5f*dt*q_ed.q2*temp->z +q_ed.q3;
}

void Yaw_to_quaternion(Quaternion *Quaternion_TEMP ,float fi ,float theta ,float gama) //欧拉角转化为四元素
{
	Quaternion_TEMP->q0=cos(0.5f*fi)*cos(0.5f*theta)*cos(0.5f*gama)+sin(0.5f*fi)*sin(0.5f*theta)*sin(0.5f*gama);
	Quaternion_TEMP->q1=sin(0.5f*fi)*cos(0.5f*theta)*cos(0.5f*gama)-cos(0.5f*fi)*sin(0.5f*theta)*sin(0.5f*gama);
	Quaternion_TEMP->q2=cos(0.5f*fi)*sin(0.5f*theta)*cos(0.5f*gama)+sin(0.5f*fi)*cos(0.5f*theta)*sin(0.5f*gama);
	Quaternion_TEMP->q3=cos(0.5f*fi)*cos(0.5f*theta)*sin(0.5f*gama)-sin(0.5f*fi)*sin(0.5f*theta)*cos(0.5f*gama);
}

Quaternion q1_q2(Quaternion *q1 ,Quaternion *q2)  //四元素q1*q2
{
	Quaternion q;
	q.q0= (q1->q0*q2->q0) - (q1->q1*q2->q1) - (q1->q2*q2->q2) - (q1->q3*q2->q3);
	q.q1= (q1->q0*q2->q1) + (q1->q1*q2->q0) + (q1->q2*q2->q3) - (q1->q3*q2->q2);
	q.q2= (q1->q0*q2->q2) + (q1->q2*q2->q0) + (q1->q3*q2->q1) - (q1->q1*q2->q3);
	q.q3= (q1->q0*q2->q3) + (q1->q3*q2->q0) + (q1->q1*q2->q2) - (q1->q2*q2->q1);
	
	return q;
}

Vector3f NED_TO_BODY(float x ,float y ,float z)
{
	Vector3f vector;
	vector.x=x*(q.q0*q.q0+q.q1*q.q1-q.q2*q.q2-q.q3*q.q3)+y*(2*(q.q1*q.q2+q.q0*q.q3))+z*(2*(q.q1*q.q3-q.q0*q.q2));
	vector.y=y*(q.q0*q.q0-q.q1*q.q1+q.q2*q.q2-q.q3*q.q3)+x*(2*(q.q1*q.q2-q.q0*q.q3))+z*(2*(q.q2*q.q3+q.q0*q.q1));
	vector.z=z*(q.q0*q.q0-q.q1*q.q1-q.q2*q.q2+q.q3*q.q3)+y*(2*(q.q3*q.q2-q.q0*q.q1))+x*(2*(q.q1*q.q3+q.q0*q.q2));
	return vector;
}

Vector3f BODY_TO_NED(float x ,float y ,float z)
{
	Vector3f vector;
	vector.x=x*(q.q0*q.q0+q.q1*q.q1-q.q2*q.q2-q.q3*q.q3)+y*(2*(q.q1*q.q2-q.q0*q.q3))+z*(2*(q.q1*q.q3+q.q0*q.q2));
	vector.y=x*(2*(q.q1*q.q2+q.q0*q.q3))+y*(q.q0*q.q0-q.q1*q.q1+q.q2*q.q2-q.q3*q.q3)+z*(2*(q.q3*q.q2-q.q0*q.q1));
	vector.z=x*(2*(q.q1*q.q3-q.q0*q.q2))+y*(2*(q.q2*q.q3+q.q0*q.q1))+z*(q.q0*q.q0-q.q1*q.q1-q.q2*q.q2+q.q3*q.q3);
	return vector;
}

float wrap_pi(float data)  //限定在-pi~~pi之间。
{
	if(data>180.0f){
	data=180;}
	if(data<-180.0f){
	data=-180;}
	return data;
}

void q_normalized(void)    //四元素归一化处理
{
	float sum=q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
	float recpiNorm=invSqrt(sum);
	q.q0*=recpiNorm;
	q.q1*=recpiNorm;
	q.q2*=recpiNorm;
	q.q3*=recpiNorm;
}

void vector_normalized(float *x,float *y,float *z)    //向量归一化处理
{
	float sum=*x**x+*y**y+*z**z;
	float recpiNorm=sqrt(sum);
	*x=*x/recpiNorm;
	*y=*y/recpiNorm;
	*z=*z/recpiNorm;
}

float math_constrain(float data ,float min ,float max)
{
	if(data>max)data=max;
	if(data<min)data=min;
	return data;
}

uint8_t is_finite(float min ,float max)
{
	if(q.q0>min&&q.q0<max&&q.q1>min&&q.q1<max&&q.q2>min&&q.q2<max&&q.q3>min&&q.q3<max)return 1;
	return 0;
}

int sign(float a, float b)     //判断符号
{
	return ((a-b)>0? 1:-1);
}

//四元数初始化耗时120us
//该处注意要将传感器的测量向量数据全部转到同NED系的右手系下
void _init(void)
{
	float ixix,jyjy,kzkz;
	
	Vector3f i,j,k;
	float mag_k=0;
	
	k.x=-senser_datas.accl_filtered_x;
	k.y=senser_datas.accl_filtered_y;
	k.z= senser_datas.accl_filtered_z;
	vector_normalized(&k.x,&k.y,&k.z);
	
	mag_k=senser_datas.mag_filtered_x*k.x - senser_datas.mag_filtered_y*k.y - senser_datas.mag_filtered_z*k.z ;
	i.x=+senser_datas.mag_filtered_x - mag_k*k.x;
	i.y=-senser_datas.mag_filtered_y - mag_k*k.y;
	i.z=-senser_datas.mag_filtered_z - mag_k*k.z;
	vector_normalized(&i.x,&i.y,&i.z);
	
	j.x=k.y*i.z-k.z*i.y;
	j.y=k.z*i.x-k.x*i.z;
	j.z=k.x*i.y-k.y*i.x;
	vector_normalized(&j.x,&j.y,&j.z);
	
	ixix=i.x*i.x;jyjy=j.y*j.y;kzkz=k.z*k.z;
	q.q0 =0.5f*sqrt(1+ixix+jyjy+kzkz);
	q.q1 =0.5f*sign(j.z,k.y)*sqrt(1+ixix-jyjy-kzkz);
	q.q2 =0.5f*sign(k.x,i.z)*sqrt(1-ixix+jyjy-kzkz);
	q.q3 =0.5f*sign(j.x,i.y)*sqrt(1-ixix-jyjy+kzkz);     //这一步还没有推导，，详见《惯性导航》秦永元
	q_normalized();
}

uint8_t update(float dt)
{
	float mag_err,sum;
	Vector3f corr={0};  //陀螺仪修正
	Quaternion q_last;
	Vector3f mag_earth; //三轴磁力计转到NED下
	Vector3f k;   //ned系下{0,0,1}向量转化到B系下
	Vector3f mag_err_NtoB; //磁偏角误差由ned系转body系下暂存向量，为减小计算资源
	float bx,bz,halfwx,halfwy,halfwz;
	
	if(system_status.attitude_estimator_q_init==0)
	{
	  _init();   //初始化四元素，第一次上电执行
		if(is_finite(-1.0f,1.0f)&&sqrt(q.q0*q.q0+q.q1*q.q1+q.q2*q.q2+q.q3*q.q3)<1.05f&&sqrt(q.q0*q.q0+q.q1*q.q1+q.q2*q.q2+q.q3*q.q3)>0.95f)              //判断四元素是否发散
	  {
		system_status.attitude_estimator_q_init=1;
	  }		
		return 0;
	}
	
	q_last=q;
	
	vector_normalized(&senser_datas.mag_filtered_x, &senser_datas.mag_filtered_y, &senser_datas.mag_filtered_z);
  //mag_earth=BODY_TO_NED(senser_datas.mag_filtered_x, (-senser_datas.mag_filtered_y), (-senser_datas.mag_filtered_z));
	mag_earth.x=senser_datas.mag_filtered_x*(q.q0*q.q0+q.q1*q.q1-q.q2*q.q2-q.q3*q.q3)+(-senser_datas.mag_filtered_y)*(2*(q.q1*q.q2-q.q0*q.q3))+(-senser_datas.mag_filtered_z)*(2*(q.q1*q.q3+q.q0*q.q2));
	mag_earth.y=senser_datas.mag_filtered_x*(2*(q.q1*q.q2+q.q0*q.q3))+(-senser_datas.mag_filtered_y)*(q.q0*q.q0-q.q1*q.q1+q.q2*q.q2-q.q3*q.q3)+(-senser_datas.mag_filtered_z)*(2*(q.q3*q.q2-q.q0*q.q1));
	mag_earth.z=senser_datas.mag_filtered_x*(2*(q.q1*q.q3-q.q0*q.q2))+(-senser_datas.mag_filtered_y)*(2*(q.q2*q.q3+q.q0*q.q1))+(-senser_datas.mag_filtered_z)*(q.q0*q.q0-q.q1*q.q1-q.q2*q.q2+q.q3*q.q3);
	
	if(system_status.GPS_alive){
	 mag_err=wrap_pi(atan2f(mag_earth.y,mag_earth.x))*3.1415926/180;    //采用弧度制
	 mag_err_NtoB=NED_TO_BODY(0,0,mag_err);
	 corr.x+=mag_err_NtoB.x*_w_mag;
	 corr.y+=mag_err_NtoB.y*_w_mag;
	 corr.z+=mag_err_NtoB.z*_w_mag;
	 q_normalized();
	 k.x=2.0f * (q.q1 * q.q3 - q.q0 * q.q2);
	 k.y=2.0f * (q.q2 * q.q3 + q.q0 * q.q1);
	 k.z=(q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);  //ned系下{0,0,1}向量转化到B系下	
	 sum=(-senser_datas.accl_filtered_x-pos_acc.x)*(-senser_datas.accl_filtered_x-pos_acc.x)
		          +(senser_datas.accl_filtered_y-pos_acc.y)*(senser_datas.accl_filtered_y-pos_acc.y)
	            +(senser_datas.accl_filtered_z-pos_acc.z)*(senser_datas.accl_filtered_z-pos_acc.z);
	 sum=invSqrt(sum);
	 corr.x +=((senser_datas.accl_filtered_y-pos_acc.y)*k.z - (senser_datas.accl_filtered_z-pos_acc.z)*k.y)*sum*_w_accl;
	 corr.y +=((senser_datas.accl_filtered_z-pos_acc.z)*k.x - (senser_datas.accl_filtered_x-pos_acc.x)*k.z)*sum*_w_accl;
	 corr.z +=((senser_datas.accl_filtered_x-pos_acc.x)*k.y - (senser_datas.accl_filtered_y-pos_acc.y)*k.x)*sum*_w_accl;
	}else{
		bx=sqrt(mag_earth.x*mag_earth.x + mag_earth.y*mag_earth.y);
		bz=mag_earth.z;
	  halfwx = bx * (0.5f - q.q2*q.q2 - q.q3*q.q3) + bz * (q.q1*q.q3 - q.q0*q.q2);
    halfwy = bx * (q.q1*q.q2 - q.q0*q.q3) + bz * (q.q0*q.q1 + q.q2*q.q3);
    halfwz = bx * (q.q0*q.q2 + q.q1*q.q3) + bz * (0.5f - q.q1*q.q1 - q.q2*q.q2);
		corr.x += ((-senser_datas.mag_filtered_y) * halfwz - (-senser_datas.mag_filtered_z) * halfwy)*_w_mag;
    corr.y += ((-senser_datas.mag_filtered_z) * halfwx - senser_datas.mag_filtered_x * halfwz)*_w_mag;
    corr.z += (senser_datas.mag_filtered_x * halfwy - (-senser_datas.mag_filtered_y) * halfwx)*_w_mag;
		
		k.x=2.0f * (q.q1 * q.q3 - q.q0 * q.q2);
	  k.y=2.0f * (q.q2 * q.q3 + q.q0 * q.q1);
	  k.z=(q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);  //ned系下{0,0,1}向量转化到B系下	
		sum=invSqrt(senser_datas.accl_filtered_x*senser_datas.accl_filtered_x + senser_datas.accl_filtered_y*senser_datas.accl_filtered_y + senser_datas.accl_filtered_z*senser_datas.accl_filtered_z);
		corr.x +=((senser_datas.accl_filtered_y)*k.z -  (senser_datas.accl_filtered_z)*k.y)*sum*_w_accl;
	  corr.y +=((senser_datas.accl_filtered_z)*k.x -  (-senser_datas.accl_filtered_x)*k.z)*sum*_w_accl;
	  corr.z +=((-senser_datas.accl_filtered_x)*k.y - (senser_datas.accl_filtered_y)*k.x)*sum*_w_accl;
	}
		
	gyro_bias.x=math_constrain(corr.x*_w_gyro_bias*dt , gyro_bias_min , gyro_bias_max);
	gyro_bias.y=math_constrain(corr.y*_w_gyro_bias*dt , gyro_bias_min , gyro_bias_max);
	gyro_bias.z=math_constrain(corr.z*_w_gyro_bias*dt , gyro_bias_min , gyro_bias_max);
	
	_rates.x=(senser_datas.gyro_filtered_x + gyro_bias.x)*3.1415926/180;
	_rates.y=(-senser_datas.gyro_filtered_y + gyro_bias.y)*3.1415926/180;
	_rates.z=(-senser_datas.gyro_filtered_z + gyro_bias.z)*3.1415926/180;   // 校正后的角速度值
	
	corr.x +=_rates.x;
	corr.y +=_rates.y;
	corr.z +=_rates.z;
		
	q_update(&corr);  //更新姿态四元素
	q_normalized();   //四元素归一化
	if(is_finite(-1.0f,1.0f)&&sqrt(q.q0*q.q0+q.q1*q.q1+q.q2*q.q2+q.q3*q.q3)<1.05f&&sqrt(q.q0*q.q0+q.q1*q.q1+q.q2*q.q2+q.q3*q.q3)>0.95f)              //判断四元素是否发散
	{
		return 1;
	}	
	q = q_last;
	memcpy(&_rates,&zero,sizeof(zero));
	memcpy(&gyro_bias,&zero,sizeof(zero));
	return 0;
}




void attitude_estimator_q_main(void)
{
	//float qqt[8];
	if(system_status.sensor_update_lock==1)  //如果正在读取传感器的值，退出（可做修改，为等待数据更新，若数据更新时间不长的话）
	{
		return;
	}
	
	//如果GPS在线可用
	if(system_status.GPS_alive){
	 if(Horizontal_Acc_Est<20.0f)
	 {
		if(Horizontal_Acc_Est<5.0f && system_status.attitude_estimator_q_init==1 && vel_prev_t!=0){
			vel_dt=(micros()-vel_prev_t)/1000000;
			pos_acc=NED_TO_BODY((GPS_Vel.N-vel_prev.N)/vel_dt,(GPS_Vel.E-vel_prev.E)/vel_dt,(GPS_Vel.D-vel_prev.D)/vel_dt);  //转化到机体坐标系下
			
			vel_prev_t=micros();
			vel_prev=GPS_Vel;
		}
		else{
			memcpy(&pos_acc,&zero,sizeof(zero));
			memcpy(&vel_prev,&zero,sizeof(zero));
			vel_prev_t=0;
	   }
   }
 }
	
	att_now_time=micros();
	dt=math_constrain((att_now_time-att_last_time)/1000000,0,0.02);
	att_last_time=att_now_time;
	if(update(dt)){
		//表示姿态解算完成，这里要干嘛还没想好就先空着
  }
}


