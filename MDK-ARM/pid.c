#include "team.h"
#include "main.h" 
#include "stm32f1xx_hal.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;


#define FF_DZ      25.0f    /* 死区 */
#define FF_GAIN    50.0f    /* 前馈增益 */

uint8_t drive_mode = 1;
//Kp,Ki,Kd,integral;,prev_err;,out_min,out_max,i_min,i_max(pid的i);
//三个参数，                   i积累 微分  最大、小输出  ，i饱和和最小大值
PID line_l = {5.0f, 0.5f,  0.0f, 0, 0, -1000, 1000, -500, 500};
PID line_r = {5.0f, 0.5f,  0.0f, 0, 0, -1000, 1000, -500, 500};
PID Rota =   {0.15f, 0.0f, 100.0f, 0, 0,  -400, 400 , -100, 100};
PID Turn =   {9.0f, 0.0f,  1.0f, 0, 0, -500 , 500 , -100, 100};
//基准速度
static float target_l = 9.0f;   /* 左轮目标 */
static float target_r = 9.0f;   /* 右轮目标 */

volatile int16_t g_spd_l = 0;
volatile int16_t g_spd_r = 0;

void PID_Init(PID *p, float kp, float ki, float kd,
              float out_min, float out_max,
              float i_min, float i_max)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->integral = 0.0f;
    p->prev_err = 0.0f;
    p->out_min = out_min;
    p->out_max = out_max;
    p->i_min = i_min;
    p->i_max = i_max;
}

void PID_Reset(PID *p)
{
    p->integral = 0.0f;
    p->prev_err = 0.0f;
}

float PID_Calc(PID *p, float err)
{
	/* --- P --- */
    float p_term = p->Kp * err;

    /* --- I：累加 + 限幅 --- */
    p->integral += err;         
    if (p->integral > p->i_max) p->integral = p->i_max;
    if (p->integral < p->i_min) p->integral = p->i_min;
    float i_term = p->Ki * p->integral;

    /* --- D：差分 --- */
    float d_term = p->Kd * (err - p->prev_err);
    p->prev_err = err;

    /* --- 求和 + 输出限幅 --- */
    float out = p_term + i_term + d_term;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    return out;
}

//包含死区与前馈，结合pid算法可以进一步减小误差，输出out
float Speed_PI(PID *p, int16_t target, float meas)//传入pid结构体参数，目标速度，还有实际测量值
{
    float err = target - meas;
	//大于0加，小于0就减
    float ff = (target >= 0.0f) ? (FF_GAIN * target + FF_DZ)
                                : (FF_GAIN * target - FF_DZ);

    float out = ff + PID_Calc(p, err);

    if (out >  1000.0f) out =  1000.0f;
    if (out < -1000.0f) out = -1000.0f;

    return out;
}

//更新速度，包含pid矫正转速，保持左右轮一致,所有更新速度的出口只用这一个,填入计数作为速度
void SpeedLoop_Update(int16_t speed_left ,int16_t speed_right)//左右轮速度
{
	
		//测量实际转速
		int16_t cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim4) ;  /* 左：TIM4 */
		int16_t cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim2) ;  /* 右：TIM2 */
	
		g_spd_l = cnt_l;                 /* ← 存一份给打印用 */
		g_spd_r = cnt_r;
	
		 __HAL_TIM_SET_COUNTER(&htim4, 0);                        //清零
		__HAL_TIM_SET_COUNTER(&htim2, 0);                         //清零
		
		//用两个变量接收out函数输出
		float out_l = Speed_PI(&line_l, speed_left  , (float)cnt_l);
		float out_r = Speed_PI(&line_r, speed_right , (float)cnt_r);

		//驱动电机
		Motor_Output_L((int)out_l);
		Motor_Output_R((int)out_r);

}
