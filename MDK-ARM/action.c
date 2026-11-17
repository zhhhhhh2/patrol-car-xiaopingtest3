/**
  ******************************************************************************
  * @file    action.c
  * @brief   四道题的动作封装：problem1 ~ problem4
  *
  *          约定：
  *            1. 所有电机输出都走 SpeedLoop_Update（速度环，单位 cnt/10ms），
  *               不直接调 Motor_Output_L/R；
  *            2. 每个 problemN 都是"顺序执行"的封装，调用后一口气跑完，
  *               适合"一键启动"的比赛场景。
  ******************************************************************************
  */
#include "team.h"
#include "main.h"
#include "key.h"
#include "led.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "sensor.h"
#include "stm32f1xx_hal.h"

#include "action.h"


/* 盲走 + 陀螺仪锁航向（原有函数，保留）
 * speed_left / speed_right 是速度环目标（每 10ms 编码器计数），不是 PWM */
void walking_blindly(int16_t speed_left ,int16_t speed_right,float target_yaw)
{
	//初始化
	float error = target_yaw - GetYaw() ;//角度误差
	int16_t corr;//修正量
	
	// 折到 ±180，yaw 过 ±180 不会错 
	while (error >  180.0f) error -= 360.0f;    
    while (error < -180.0f) error += 360.0f;
	
	//偏差过大
	if (error >  30.0f) { SpeedLoop_Update(-speed_left, speed_right); return; }  /* 偏差大：原地左转 */
    if (error < -30.0f) { SpeedLoop_Update(speed_left, -speed_right); return; }  /* 偏差大：原地右转 */
	
	//偏差小，左右轮略微加一点转速修正
	corr = (int16_t)(2.0f * error);           // 1° 误差 → 左右差 2 个计数 ,调参数用这个
    if (corr >  speed_left)  corr =  speed_left;
    if (corr < -speed_left)  corr = -speed_left;
	
	// 差速修 
	SpeedLoop_Update(speed_left - corr, speed_right + corr);   
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


/* ======================= 可调参数 ======================= */

#define A_SPEED        9       /* 直行速度（cnt/10ms，9≈19rpm） */
#define A_STRAIGHT_MS  1500    /* 直走一段的时长（ms）——按场地实测改 */
#define A_LAP_TOTAL    4       /* 第 4 题跑几圈 */


/* ======================= 三个小动作 ======================= */

/* PWM(0~1000) → 速度环目标(cnt/10ms)：按标定 delta = 0.02*(pwm-25) */
static int16_t A_PwmToSpeed(int16_t pwm)
{
    float s = ((float)pwm - 25.0f) / 50.0f;

    if (s < 0.0f)  { s = 0.0f; }
    if (s > 20.0f) { s = 20.0f; }
    return (int16_t)s;
}


/* 循迹到丢线（弯道段）：Sensor_Find 出 PWM → 速度环 */
static void A_Follow(void)
{
    uint16_t l, r;
    uint8_t  lost = 0;

    while (lost < 3)                                     /* 连续 3 拍丢线 = 出弯 */
    {
        Sensor_Find(&l, &r);                             /* 红外 → 左右 PWM */
        SpeedLoop_Update(A_PwmToSpeed((int16_t)l), A_PwmToSpeed((int16_t)r));
		
        if (Sensor_GetState() == SENSOR_STATE_LOST) { lost++; }
        else                                        { lost = 0; }
        HAL_Delay(10);
    }
    SpeedLoop_Update(0, 0);                              /* 停车 */
}

/* 原地转 delta 度（+左 −右），转到 ±2° 就停 */
static void A_Turn(float delta)
{
    float target = GetYaw() + delta;

    while (1)
    {
        float err = target - GetYaw();

        if (err >  180.0f) { err -= 360.0f; }
        if (err < -180.0f) { err += 360.0f; }

        if (err < 2.0f && err > -2.0f) { break; }
        Rota_Update(target, GetYaw());                   /* 原地转 */
        HAL_Delay(10);
    }
    SpeedLoop_Update(0, 0);                              /* 停车 */
}


/* ======================= 第 1 题 ======================= */
/* 现实步骤：从 A 点出发 → 沿直线盲走到 B 点 → 停车。 */
void problem1(void)
{
    A_Straight(0.0f, A_STRAIGHT_MS);   /* 步骤1：A → B 盲走 */
    Beep(BEEP_LONG_MS);                /* 步骤2：到 B 响一声 */
}


/* ======================= 第 2 题 ======================= */
/* 现实步骤：A→B 直行 → B 弯循迹 → C→D 直行 → D 弯循迹 → 回 A 停车。 */
void problem2(void)
{
    float yaw_cd;

    A_Straight(0.0f, A_STRAIGHT_MS);    /* 步骤1：A → B 盲走 */
    A_Follow();                          /* 步骤2：B 弯循迹，出弯为止 */
    yaw_cd = GetYaw();                   /* 记下出弯后的车头方向 */
    A_Straight(yaw_cd, A_STRAIGHT_MS);  /* 步骤3：C → D 盲走 */
    A_Follow();                          /* 步骤4：D 弯循迹，出弯为止 */
    Beep(BEEP_LONG_MS);                  /* 步骤5：回到 A 停车 */
}


/* ======================= 第 3 题 ======================= */
/* 现实步骤：A 左转 90° → A→C 直行 → C 弯循迹 → B 左转 90° → B→D 直行 → D 弯循迹 → 回 A 停车。 */
void problem3(void)
{
    float yaw;

    A_Turn(90.0f);                      /* 步骤1：A 点左转 90°，对准 C */
    yaw = GetYaw();
    A_Straight(yaw, A_STRAIGHT_MS);     /* 步骤2：A → C 盲走 */
    A_Follow();                          /* 步骤3：C 弯循迹 */
    A_Turn(90.0f);                      /* 步骤4：B 点再左转 90°，对准 D */
    yaw = GetYaw();
    A_Straight(yaw, A_STRAIGHT_MS);     /* 步骤5：B → D 盲走 */
    A_Follow();                          /* 步骤6：D 弯循迹 */
    Beep(BEEP_LONG_MS);                  /* 步骤7：回 A 停车 */
}


/* ======================= 第 4 题 ======================= */
/* 现实步骤：第 2 题的流程，连续跑 4 圈，最后回 A 停车。 */
void problem4(void)
{
    uint8_t lap;
    float   yaw_cd;

    for (lap = 0; lap < A_LAP_TOTAL; lap++)
    {
        A_Straight(0.0f, A_STRAIGHT_MS);    /* 步骤1：A → B 盲走 */
        A_Follow();                          /* 步骤2：B 弯循迹 */
        yaw_cd = GetYaw();
        A_Straight(yaw_cd, A_STRAIGHT_MS);  /* 步骤3：C → D 盲走 */
        A_Follow();                          /* 步骤4：D 弯循迹 */
    }
    Beep(BEEP_LONG_MS);                      /* 步骤5：跑满 4 圈停车 */
}
