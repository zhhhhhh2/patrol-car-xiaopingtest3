/**
  ******************************************************************************
  * @file    task.h
  * @brief   比赛状态机（把队友 user\main.c 里的状态机整段剥离到这里）
  *
  *          main.c 里只要：
  *              Task_Init();                  // USER CODE 2 里
  *              while (1) { Task_Update(); }  // 主循环里
  ******************************************************************************
  */
#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>
#include "sensor.h"
#include "team.h"

//对于不同模式进行定义
typedef enum {
    TASK_RUN_PROBLEM_1,//0
    TASK_RUN_PROBLEM_2,//1
    TASK_RUN_PROBLEM_3,//2
    TASK_RUN_PROBLEM_4,//3
	TASK_RUN_PROBLEM_5,//4
	TASK_RUN_PROBLEM_6,//5
	TASK_RUN_PROBLEM_7,//6
	TASK_RUN_PROBLEM_8,//7
    TASK_STOP          //停止
} RunTask_t;

extern RunTask_t current_task;

//状态机与计数变量
extern uint8_t point_count;           // 记录经过点的次数
extern uint8_t point_key_count;       //记录按键按下次数

extern SensorState_t current_sensor_state;
extern SensorState_t last_sensor_state; // 由于发车时在直道上，所以最开始的上一次状态为全白状态
extern uint8_t state_change_cnt;      // 状态切换计数器

extern uint8_t s_val[7];              //用来存放七个传感器数据
extern float line_error;              //加权平均后的行驶偏差
extern int16_t base_speed;            //基础速度后面可以修改
extern int16_t pwm_L;
extern int16_t pwm_R;

uint8_t Check_Point_Event(void);

void Task_Init(void);                 // 只复位状态，硬件初始化在 main.c 里做
void Task_Update(void);               // 原来 main() 的 while(1) 里的那一大段

#endif /* __TASK_H */
