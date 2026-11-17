/**
  * @file    turn.c
  * @brief   原地转向 + 陀螺仪接入（队友 B 的文件，这里只补了 BSP_IMU_GetYaw 的实现）
  *
  * 本次改动（其余一个字没动）：
  *   1) BSP_IMU_GetYaw() 原来是个空壳（永远返回 0，PID 根本转不动），
  *      现在直接返回 MPU6050_YawData()（-180 ~ +180 度）。
  *      前提：main.c 里调了 MPU6050_Init()，并且 Task_Update() 每 10ms 调一次
  *      MPU6050_Proc()（task.h 里 TASK_IMU_ENABLE 默认就是 1）。
  *   2) 原来的 turn_Update() 写成了 `Sensor_Find(target_turn, int16_t *pwm_L, ...)`
  *      这种非法的参数写法，turn.c 在工程里是参与编译的，会直接报语法错误。
  *      先用 #if 0 关掉，以后要"边走边修正角度"再按真正的写法打开。
  */
#include "team.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "mpu6050.h"                 /* 新增：陀螺仪 */
#include <math.h>                   /* fabsf */
#include "sensor.h"

extern PID Turn;
extern PID Rota;

//自转基础速度
#define target_turn (12)

//角
//是否使用pid
#define use_pid (1) //0为不用

#define FF_DZ      25.0f    /* 死区 */
#define FF_GAIN    50.0f    /* 前馈增益 */

//循迹
#define turn_base_speed (900)

/*
PWM	计数/10ms（delta）	折算 rpm 最高大概是20-21之间
100	  1~2                  2~4
200	  3~4	               6~8
400	  8	                   17
600	  12	               25
800	  16	               34
1000  20	               42.3
*/
/*--------------------------------------------------------------*/
//从A 指向 C：\(\boldsymbol{\approx +30.96^\circ}\)（约 50.2°）
//B 指向 D：\(\boldsymbol{\approx -129.81^\circ}\)（约 - 143.1°）
/* 陀螺仪 yaw：范围 -180 ~ +180 度，来自 mpu6050.c 的角速度积分 */
float GetYaw(void)
{
	return MPU6050_YawData();
}

void Rota_Update(float target_yaw, float current_yaw)
{
	//和目标的误差
	float error = target_yaw - current_yaw;
	float out;
	
	//陀螺仪反转，误差同时取反抵消
	error = - error;
	
	//角度归一化
	//关键：折到 ±180，否则 yaw 过 ±180 时 error 会跳 360°，
	//输出瞬间反号 -> 先冲过头、再反向一直转 
	while (error >  180.0f) error -= 360.0f;
	while (error < -180.0f) error += 360.0f;

	if (error < 1.0f && error > -1.0f)
	{
		SpeedLoop_Update(0,0);
		return;
	}
	/*if( use_pid )
	{
		out = PID_Calc( &Rota , error );
	}
	else
	{*/
		//pid暂时不用，调不好
		float abs_err = fabs(error);//取绝对值
		if      (abs_err > 10.0f) out = 12;   // 大误差：全速
		else if (abs_err > 5.0f) out = 10;   // 中误差：中速
		else if (abs_err > 1.0f) out = 7;   // 小误差：慢速
		//else if (abs_err > 3.0f) out = 3;
		
		if (error < 0)
		{
			out = -out;//设置自转速度
		}
		

	/* 原地转：左反右正。若越转 yaw 越反，把下面两行正负对调 */
	SpeedLoop_Update((int)(-out),(int)(out));
}

/* ----------------------------------------------------------------*/
//循迹转弯
void turn_Update(PID *p)
{
		//速度初始化
		float add_out = 0;
		int16_t left_speed  = turn_base_speed;
		int16_t right_speed = turn_base_speed;
		
		//传感器获取误差
		float error = Sensor_GetError() * 100.0f;
		
		//pid计算调控输出
		add_out = PID_Calc( &Turn , error );
		
		//pwm和计数换算
		int16_t left_cnt  = (left_speed - add_out - 25) / 50;
		int16_t right_cnt = (right_speed + add_out- 25) / 50;
		//驱动电机
		SpeedLoop_Update( left_cnt , right_cnt);
}




















