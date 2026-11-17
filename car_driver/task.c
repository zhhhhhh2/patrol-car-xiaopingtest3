/**
  ******************************************************************************
  * @file    task.c
  * @brief   比赛状态机 —— 内容来自队友 C:\Users\zh\Desktop\灰度传感器寻迹6\user\main.c
  *
  *          原来 main() 里的 while(1) 整段移到 Task_Update()，
  *          原来 main() 开头的 Serial_Init()/Sensor_Init() 留在 main.c 里。
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "sensor.h"
#include "team.h"
#include "task.h"

//核心配置选择触发模式 
#define MODE_ON_LOST_TO_LINE  0  // 从全白进入黑线 
#define MODE_ON_LINE_TO_LOST  1  // 从黑线进入全白

// 选择你需要的模式
#define CURRENT_MODE     MODE_ON_LINE_TO_LOST  

//连续识别五次，一次5ms，五次就是25ms，即每次确定是否检测到点至少需要25ms
#define DEBOUNCE_THRES           5  

RunTask_t current_task = TASK_RUN_PROBLEM_1;

//状态机与计数变量
uint8_t point_count = 0;           // 记录经过点的次数
uint8_t point_key_count=0;         //记录按键按下次数

SensorState_t current_sensor_state = SENSOR_STATE_LOST; 
SensorState_t last_sensor_state = SENSOR_STATE_LOST; // 由于发车时在直道上，所以最开始的上一次状态为全白状态
uint8_t state_change_cnt = 0;      // 状态切换计数器

uint8_t s_val[7];          //用来存放七个传感器数据
float line_error = 0.0f;   //加权平均后的行驶偏差
int16_t base_speed=500;    //基础速度后面可以修改
int16_t  pwm_L=0;
int16_t  pwm_R=0;


uint8_t Check_Point_Event(void)
{
    // 获取当前任务状态
    current_sensor_state = Sensor_GetState();
    
    // 状态发生改变
    if (current_sensor_state != last_sensor_state) {
        state_change_cnt++;
        
        // 连续多次确认状态真的改变了
        if (state_change_cnt >= DEBOUNCE_THRES) {
            state_change_cnt = 0; // 计数器清零
            
            // 记录触发前一刻的状态，用于后面判断
            SensorState_t prev_state = last_sensor_state; 
            last_sensor_state = current_sensor_state; // 更新上一次状态
            
            // 修复：原来这里用的是没定义过的宏 CURRENT_TRIGGER_MODE / TRIGGER_ON_LINE_TO_LOST
            //       （未定义的宏按 0 算，0==0 恒成立），现在按选定的 CURRENT_MODE 判断
            if (CURRENT_MODE == MODE_ON_LINE_TO_LOST)
            {
                if (prev_state == SENSOR_STATE_NORMAL && current_sensor_state == SENSOR_STATE_LOST) 
                {
                    return 1; // 有黑线到全白，触发事件
                }
            }
            else
            {
                if (prev_state == SENSOR_STATE_LOST && current_sensor_state == SENSOR_STATE_NORMAL)
                {
                    return 1; // 全白到有黑线，触发事件
                }
            }
        }
    } else {
        state_change_cnt = 0; // 状态没变，次数清零
    }
    
    return 0; // 无事件
}

/**
  * @brief  原来 main() 开头那两行状态初始化
  */
void Task_Init(void)
{
    current_task = TASK_RUN_PROBLEM_1;     //初始状态的模式为模式一
    point_count = 0;
    point_key_count = 0;
    state_change_cnt = 0;
    current_sensor_state = SENSOR_STATE_LOST;
    last_sensor_state = SENSOR_STATE_LOST;
}

/**
  * @brief  原来 main() 里 while(1) 的那一大段
  */
void Task_Update(void)
{
    Sensor_ReadArray(s_val);     //将七个传感器数据读到s_val[7]中
    line_error = Sensor_GetError();

    // --- 2. 检测状态切换，判定是否经过点 ---
    if (Check_Point_Event() == 1) 
    {
        point_count++;  // 成功经过一个点
        
        // 串口打印说明是否识别到点
        UART_SendStr("Point Event Count: ");
        UART_SendInt((int16_t)point_count);
        UART_SendStr("\r\n");
        
        // 根据经过点的次数，切换任务
        if (point_count == 1) 
        {
            current_task = TASK_RUN_PROBLEM_2; 
        } 
        else if (point_count == 2) 
        {
            current_task = TASK_RUN_PROBLEM_3; 
        } 
        else if (point_count == 3)
        {
            current_task = TASK_RUN_PROBLEM_4;          
        }
        else if (point_count == 4)
        {
            current_task = TASK_RUN_PROBLEM_5;          
        }
        else if (point_count == 5)
        {
            current_task = TASK_RUN_PROBLEM_6;          
        }
        else if (point_count == 6)
        {
            current_task = TASK_RUN_PROBLEM_7;          
        }
        else if (point_count == 7)
        {
            current_task = TASK_RUN_PROBLEM_8;          
        }
    }

    //任务执行状态机
	if(point_key_count==1)      //按键按下次数等于一就是第一题        总时长不超过15秒
	{ 
		switch (current_task)
        {   
            
            case TASK_RUN_PROBLEM_1 :;   //模式一是盲走阶段
            {
                //陀螺仪控制
                
                break;
            }

            
            case TASK_RUN_PROBLEM_2 :;   //模式二是第一次识别到黑线
            {
                //第一题要求到B这里停车，就在这写停车
   
				//加上声光提示                                            
                
                // 停一段时间后继续跑第二题的控制逻辑
                break;
            }
		}
        // 修复：原来这段末尾有 point_count=0; 每轮都会清零，导致永远到不了第 2 个点；
        //       现在只在 Task_Init() 里清零，四段 if 末尾的清零都删掉了
	}
		
	if(point_key_count==2)    //第二题
    {
		switch(current_task)
		{
			case TASK_RUN_PROBLEM_1:;  //盲走阶段
			{
				//正常寻迹加上陀螺仪控制
				break;
			}
			
			case TASK_RUN_PROBLEM_2:;      //此时到达B点进入寻迹转弯阶段
        {    //先声光提示
			Sensor_Find(&pwm_L,&pwm_R);     //转弯的时候基础速度可以降一点
			//继续加电机，陀螺仪等函数
			break;
				
		}	
		case TASK_RUN_PROBLEM_3 :;   //第二题第二次过点，此时经过C点出弯道
        {    //先声光提示
		   //这里最好用陀螺仪来锁死航向加上控制左右轮转速相同来确保走直线
			break;
		}
		
		case TASK_RUN_PROBLEM_4 :;   //第二题第三次过点，此时经过D点进入弯道
		{   //声光提示
			Sensor_Find(&pwm_L,&pwm_R);
            	//继续加电机，陀螺仪等函数
               break;			
		}
		
		case TASK_RUN_PROBLEM_5:;   //第二题第四次过点，此时到达A点停车
		{
			//先声光提示
			//电机停车代码
			break;
		}
		}
	}
	
	if(point_key_count==3)       //按键第三次按下，此时进入第三题
	{
		switch(current_task)
		{
			case TASK_RUN_PROBLEM_1 :;   //此时小车将从A点转向后直走到C点
			{
				//此时用手动根据实际情况对左右电机转速进行赋值进行转向
				//转向后按照前面盲走阶段代码写，用陀螺仪锁死方向之后控制电机转速直走到C点
				break;
	
		    }
			case TASK_RUN_PROBLEM_2 :;   //此时小车到达C点进入弯道
			{   
				//声光提示
				Sensor_Find(&pwm_L,&pwm_R);
				//继续加电机陀螺仪函数
				break;
			}
			case TASK_RUN_PROBLEM_3:;     //此时小车到达B点
			{
				//此时用手动根据实际情况对左右电机转速进行赋值进行转向
				//转向后按照前面盲走阶段代码写，用陀螺仪锁死方向之后控制电机转速直走到C点
				break;
			}
			case  TASK_RUN_PROBLEM_4 :;    //此时小车到达D点进入弯道
			{
				//声光提示
				Sensor_Find(&pwm_L,&pwm_R);
				//继续加电机陀螺仪函数
				break;
			}
			case TASK_RUN_PROBLEM_5  :;    //此时小车回到A点
			{
				//声光提示
				//停车代码
				break;
			}
		}
	}
	
	if(point_key_count==4)         //此时按键按下第四次，进入第四题
	{
        // 修复：原来是 for (int i=0;i<3;i++) { switch ... }，i 从不参与判断，等于没有循环；
        //       现在用 point4_lap 计圈，回到 A 点后不满 4 圈就重新从 PROBLEM_1 开始
		switch(current_task)
		{
				case TASK_RUN_PROBLEM_1 :;   //此时小车将从A点转向后直走到C点
			{
				//此时用手动根据实际情况对左右电机转速进行赋值进行转向
				//转向后按照前面盲走阶段代码写，用陀螺仪锁死方向之后控制电机转速直走到C点
				break;
	
		    }
			case TASK_RUN_PROBLEM_2 :;   //此时小车到达C点进入弯道
			{   
				//声光提示
				Sensor_Find(&pwm_L,&pwm_R);
				//继续加电机陀螺仪函数
				break;
			}
			case TASK_RUN_PROBLEM_3  :;   //此时小车到达B点
			{
				//此时用手动根据实际情况对左右电机转速进行赋值进行转向
				//转向后按照前面盲走阶段代码写，用陀螺仪锁死方向之后控制电机转速直走到C点
				break;
			}
			case TASK_RUN_PROBLEM_4 :;     //此时小车到达D点进入弯道
			{
				//声光提示
				Sensor_Find(&pwm_L,&pwm_R);
				//继续加电机陀螺仪函数
				break;
			}
			case TASK_RUN_PROBLEM_5 :;     //此时小车回到A点
			{
				//声光提示	
				break;
			}
		}
        // 修复：删掉了原来在 for 循环后面又重复一遍的那整段 switch（第 4 题每轮会连跑两遍）
		if (current_task == TASK_RUN_PROBLEM_5)
        {
            static uint8_t point4_lap = 0;   // 修复：原来 for(i<3) 不生效，改用圈数计数
            point4_lap++;
            if (point4_lap < 4)
            {
                current_task = TASK_RUN_PROBLEM_1;
                point_count = 0;
            }
        }
	}

    // 修复：原来 main() 的 while(1) 里没有延时，5 次消抖实际≈0ms；
    //       这里补 5ms，使消抖窗口≈25ms（与上面 DEBOUNCE_THRES 的注释一致）
    HAL_Delay(5);
}
