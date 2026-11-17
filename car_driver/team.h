#ifndef __team_H__
#define __team_H__

#include <stdint.h>
#include "main.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"

typedef struct
{
	float Kp,Ki,Kd;
	float integral;
	float prev_err;
	float out_min,out_max;
	float i_min,i_max;
}PID; //pid结构体

/* ---- 函数声明：放这里 ---- */
void  PID_Init(PID *p, float kp, float ki, float kd,
               float out_min, float out_max,
               float i_min, float i_max);
void  PID_Reset(PID *p);
float PID_Calc(PID *p, float err);
			   
extern PID line_l;
extern PID line_r;
extern PID Rota;
extern PID Turn;
			   
/*-------------------pid算法----------------------*/		   

float PID_Calc(PID *p, float err);

/*---------------------电机驱动-------------------*/
			   
//驱动左电机
void Motor_Output_L(int pwm);
	
//驱动右电机
void Motor_Output_R(int pwm);
	
//停车
void stop(void);


void walking_blindly(int16_t speed_left ,int16_t speed_right,float target_yaw);
/*--------------------pid更新速度-------------------*/
//下面两个是配套的
//单轮 PI + 前馈
float Speed_PI(PID *p, int16_t target, float meas);

//速度循环更新
void SpeedLoop_Update(int16_t speed_left ,int16_t speed_right);

//重置pid
void PID_Reset(PID *p);

/*--------------------pid更新速度-------------------*/

void Rota_Update(float target_yaw, float current_yaw);


/*--------------------自转----------------------*/	

// 陀螺仪 yaw：范围 -180 ~ +180 度，来自 mpu6050.c 的角速度积分 
float GetYaw(void);

//转到目标角度
void Rota_Update(float target_yaw, float current_yaw);

/*--------------------循迹转弯----------------------*/
//双重pid，未调参数
void turn_Update(PID *p);

/*--------------------串口调试----------------------*/

// 一次性发送这4个调试变量 
void Debug_Send4Int(int16_t lcnt, int16_t rcnt, int16_t lpwm, int16_t rpwm);

//串口发送调试
void UART_SendStr(const char *s);

// 发送 int16_t 整数，支持正负 
void UART_SendInt(int16_t num);

void PID_Reset_all(void);
#endif

