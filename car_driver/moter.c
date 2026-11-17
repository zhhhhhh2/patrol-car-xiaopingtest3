#include "team.h"
#include "stdio.h"
#include "main.h" 
#include "stm32f1xx_hal.h"
#include "math.h"
extern TIM_HandleTypeDef htim1;


//驱动左电机
void Motor_Output_L(int pwm)
{
    if (pwm > 0) {                                        /* 左轮前进 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  /* BIN1=0 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);    /* BIN2=1 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)pwm);
    } else if (pwm < 0) {                                 /* 左轮后退 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(-pwm));
    } else {                                              /* 停 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

//驱动右电机
void Motor_Output_R(int pwm)
{
if (pwm > 0) {                                        /* 右轮前进 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);  /* AIN1=0 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);    /* AIN2=1 */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);
} else if (pwm < 0) {                                 /* 右轮后退 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-pwm));
} else {                                              /* 停 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
}

//停车
void stop(void)
{
	
	Motor_Output_L(0);
	Motor_Output_R(0);
	PID_Reset(&line_l);
	PID_Reset(&line_r);
	PID_Reset(&Turn);
	PID_Reset(&Rota);
}

void PID_Reset_all(void)
{
	PID_Reset(&line_l);
	PID_Reset(&line_r);
	PID_Reset(&Turn);
	PID_Reset(&Rota);
}

void walking_blindly(int16_t speed_left ,int16_t speed_right,float target_yaw)
{
	float   error = target_yaw - GetYaw();
	int16_t corr, l, r;

	/* ① 先归一化，必须在所有判断之前 */
	while (error >  180.0f) error -= 360.0f;
	while (error < -180.0f) error += 360.0f;

	/* ② 死区 */
	if (fabsf(error) < 2.0f)
	{
		SpeedLoop_Update(speed_left, speed_right);
		return;
	}

	/* ③ 大偏差原地转：提高门槛到 60°，并且不再用 static 锁存 */
	if (fabsf(error) > 60.0f)
	{
		if (error > 0.0f) SpeedLoop_Update(-4,  4);
		else              SpeedLoop_Update( 4, -4);
		return;
	}

	/* ④ 小偏差差速修正 */
	corr = (int16_t)(2.0f * error);
	if (corr >  6) corr =  6;
	if (corr < -6) corr = -6;

	l = speed_left  + corr;
	r = speed_right - corr;
	if (l < 0) l = 0;
	if (r < 0) r = 0;

	SpeedLoop_Update(l, r);
}

//转弯（第三题）
void spot_turn(int16_t speed_left ,int16_t speed_right,float target_yaw)//启动时需要传入初始速度（转弯速度可以慢）
{
	target_yaw = 38.6598;
	float error = target_yaw - GetYaw();
	
	if(error > 0) { SpeedLoop_Update(-speed_left, speed_right); return; };
	if(error < 0) { SpeedLoop_Update(speed_left, -speed_right); return; };
}

/*

主要要用的函数

获取当前角度
float GetYaw(void)
自转矫正
void Rota_Update(float target_yaw, float current_yaw)
循迹转弯
void turn_Update(PID *p)
pid驱动电机
void SpeedLoop_Update(int16_t speed_left ,int16_t speed_right)
非阻塞按钮
uint8_t Key_Scan(uint16_t debounce_ms)
蜂鸣器
void Beep(uint16_t ms)
灯
void LED_ON(void)

void LED_OFF(void)

void LED_Turn(void)

*/

