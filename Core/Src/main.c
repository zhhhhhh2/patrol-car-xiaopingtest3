/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "team.h"           /* 队友 B 的电机/编码器/PID/串口接口 */
#include "math.h"
/* 队友 A 的硬件驱动 + 状态机（已从标准库改成 HAL，见 car_driver 目录） */
#include "sensor.h"         /* 7 路灰度循迹传感器 */
#include "key.h"            /* 启动按键 */
#include "led.h"            /* LED + 蜂鸣器 */
#include "task.h"           /* 状态机（原来塞在 main.c 里，现在独立成 task.c） */
#include "mpu6050.h"        /* 陀螺仪（队友 C 的 MPU6050，走硬件 I2C1：PB8/PB9） */
extern volatile int16_t g_spd_l, g_spd_r;   /* 文件顶部 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* ============================================================
 *  运行状态打印：每 100ms 输出一行小车状态
 *
 *  格式：L=左轮计数  R=右轮计数  yaw=角度(0.1°)  err=灰度误差(0.01)
 *  例：  L=9 R=9 yaw=-1234 err=250
 *        → 左右轮各 9 计数/10ms，当前 -123.4°，灰度误差 2.50
 *
 *  放在 main 的 while(1) 里，自检打印之后调用即可。
 * ============================================================ */
void Car_State_Print(void)
{
    static uint32_t t0 = 0;

    if (HAL_GetTick() - t0 < 100U) return;   /* 10Hz，不刷屏 */
    t0 = HAL_GetTick();

    UART_SendStr("L=");
    UART_SendInt(g_spd_l);

    UART_SendStr(" R=");
    UART_SendInt(g_spd_r);

    UART_SendStr(" yaw=");
    UART_SendInt((int16_t)(GetYaw() * 10.0f));     /* ×10，0.1° 精度 */

    UART_SendStr(" err=");
    UART_SendInt((int16_t)(Sensor_GetError() * 100.0f));  /* ×100，0.01 精度 */

    UART_SendStr("\r\n");
}

void mpu_recalibrate(uint16_t number)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	stop();
	
	//重新标定
	MPU6050_Calibrate(number);           
	MPU6050_ResetYaw();  
	
	HAL_TIM_Base_Start_IT(&htim3);
}
//异常终止
void fault_stop(void)
{
	stop();
	LED_ON();
	Buzzer_ON();
	HAL_Delay(800);
	Buzzer_OFF();
	while (1) { HAL_Delay(100); }      // 停在这里，等人复位 
}

//万一放偏丢线,原地自转一圈找线
uint8_t find_line(uint8_t direction)       //direction: 0是逆时针，1是顺时针，直接传入参数就可以改变找线方向
{
	float    prev = GetYaw();
	float    acc  = 0.0f;
	float    d;
	uint8_t  ok   = 0;
	uint32_t t0   = HAL_GetTick();

	int16_t left_speed  = (direction == 0) ? -12 : 12;
	int16_t right_speed = (direction == 0) ?  12 : -12;

	while (acc < 360.0f)
	{
		/* ① 绝对超时：正常一圈约 4s，给一倍余量 */
		if (HAL_GetTick() - t0 > 8000U)
		{
			stop();
			return 0;
		}

		SpeedLoop_Update(left_speed, right_speed);

		/* ② 连续 3 拍确认，屏蔽单拍噪声 */
		if (Sensor_GetState() != SENSOR_STATE_LOST)
		{
			if (++ok >= 3)
			{
				stop();
				return 1;
			}
		}
		else
		{
			ok = 0;
		}

		HAL_Delay(10);

		/* ③ 累计转角（±180 折返修正） */
		d = GetYaw() - prev;
		if (d >  180.0f) d -= 360.0f;
		if (d < -180.0f) d += 360.0f;
		acc  += fabsf(d);
		prev  = GetYaw();
	}

	stop();
	return 0;
}



void drive_stright_stop(float target_yaw,uint8_t problem,uint8_t direction)
{	
	uint16_t ok = 0;
	uint32_t start_time = HAL_GetTick();    //记录开始盲走的时间

	while (1)
	{
		uint32_t run_time=HAL_GetTick()-start_time;   //单位是毫秒
		if (problem == 1 || problem == 2)
		{
			walking_blindly(19, 19, target_yaw);      /* 电机：保持 10ms 一拍，不能动 */
		}
		else
		{
			
			if (run_time >= 7800)                             // 8.7 秒减速
			{
				walking_blindly(14, 14, target_yaw); 
			}
			else
			{
				walking_blindly(19, 19, target_yaw);      /* 电机：保持 10ms 一拍，不能动 */
			}
		}
		
		if(run_time>=11000)
		{
			stop();
			if(find_line(direction)==1)
			{
				return;
			}
			stop();
			
			while (1)
			{   
				LED_ON();
				Beep(80); 
				HAL_Delay(80);
				Beep(80); 
				HAL_Delay(80);	
				Beep(80);                // 三声急响 表示 出错
				LED_OFF();
				//死机，等待人工复位
			}
		}
		
		if (Sensor_GetState() != SENSOR_STATE_LOST) ok++;
		else                                        ok = 0;
		
		if (ok >= 3)     { stop(); break; }       /* 连续 ≈30ms 看见线 → 真线 */
		
		HAL_Delay(10);
	}
}

void turn_start_confirm(float target_yaw)//和直走的方向塞入同一个角度即可
{
	int ok = 0;
	while (ok < 10)
	{
		walking_blindly(19, 19, target_yaw);    // 继续直走 
		if (Sensor_GetState() == SENSOR_STATE_NORMAL) ok++;
		else ok = 0;
		HAL_Delay(10);
	}
}
void turn_finished_confirm(void)
{
	int lost = 0;
	while (lost < 16)
	{
		turn_Update(&Turn);
		if (Sensor_GetState() == SENSOR_STATE_LOST) 
		{
			lost++;
		}
		else lost = 0;
		HAL_Delay(10);
	}
}

void turn_finished_Calibrate()
{
	// 直走一段确保车走出来转弯后可以对齐直线
	for (int i = 0 ;i<135;i++)
	{
		// 先不用盲走直接用速度更新试一下，不行再换
		SpeedLoop_Update(14 ,14);
		HAL_Delay(10);
	}
	PID_Reset_all();
}

void continue_rota(float target_yaw)
{
	uint16_t t = 0;//时间计数
	
	while(1)
	{
		float e = target_yaw - GetYaw();
		//角度归一
		if (e > 180.0f)  e -= 360.0f;
		if (e < -180.0f) e += 360.0f;
		
		if(fabsf(e) < 1.0f) break;
		if(t++ >= 800) break;
		
		Rota_Update(target_yaw, GetYaw());
		HAL_Delay(10);
	}
	
	stop(); //停下顺便清掉速度环积分
}

void voice_light(void)
{
	LED_ON();
	Beep(300);
	LED_OFF();
}

/*--------------------------------------------------------*/
 //direction: 0是逆时针，1是顺时针，直接传入参数就可以改变找线方向
void problem1(void)
{ 
	// 直走直到遇到黑线
	drive_stright_stop(0.0f,1,0);
	// 停车
	stop();
}

void problem2(void)
{
    // 盲走到看见线 
	drive_stright_stop(0.0f,2,0);
	voice_light();
	
	// 线出现后再确认 3 拍，防止不会因为轨迹卡在两个传感器之间直接切到下一个步骤
	turn_start_confirm(0.0f);
	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	// 直行一点再转对准直线
	turn_finished_Calibrate();
	voice_light();
	// 自转矫正角度
	continue_rota(180.0f);
	
	// 盲走到看见线 
	drive_stright_stop(180.0f,2,0);
	voice_light();
	
	
	// 线出现后再确认 8 拍，防止不会因为轨迹卡在两个传感器之间直接切到下一个步骤
	turn_start_confirm(180.0f);
	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	// 停车
	stop();
}

void problem3(void)
{
	// 自转矫正角度
	continue_rota(-38.66f);
	
	
    // 盲走到看见线 
	drive_stright_stop(-38.66f,3,1);//-38.66f测试的时候记得改
	voice_light();

	
	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	voice_light();
	// 直行一点再转
	turn_finished_Calibrate();
	// 自转矫正角度
	continue_rota(-143.25f);//143.0f


	// 盲走到看见线 
	drive_stright_stop(-143.25f,3,0);
	voice_light();


	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	stop();
}
// 停车stop();34后面做完记得加因为要循环调用所以不能写在里面声光提示voice_light();
void problem4()
{
	problem3();
	for (int i = 0;i<3;i++)
	{
		turn_finished_Calibrate();
		problem3();
	}
}

//---------------------附加题-----------------------------//
void route_l(void)
{
	//初始自转调角度
	continue_rota(0.0f);
	// 盲走到看见线 
	drive_stright_stop(0.0f,2,0);

	// 线出现后再确认 10 拍，防止不会因为轨迹卡在两个传感器之间直接切到下一个步骤
	turn_start_confirm(0.0f);
	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	// 直行一点再转对准直线
	turn_finished_Calibrate();

	// 自转矫正角度
	continue_rota(180.0f);
	
	// 盲走到看见线 
	drive_stright_stop(180.0f,2,0);

	// 线出现后再确认 8 拍，防止不会因为轨迹卡在两个传感器之间直接切到下一个步骤
	turn_start_confirm(180.0f);
	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	
	// 直行一点再转对准直线以便于转弯
	turn_finished_Calibrate();
}

void route_x(void)
{
	// 自转矫正角度
	continue_rota(-38.66f);
	
	
    // 盲走到看见线 
	drive_stright_stop(-38.66f,3,1);//-38.66f测试的时候记得改


	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	// 直行一点再转
	turn_finished_Calibrate();
	// 自转矫正角度
	continue_rota(-143.25f);//143.0f


	// 盲走到看见线 
	drive_stright_stop(-143.25f,3,0);


	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	
	// 直行一点再转对准直线以便于转弯
	turn_finished_Calibrate();
}

char get_data[3];

//解析接收到的数据
uint8_t data_analysis(const char*s, uint8_t n)
{
	uint8_t i;
	char c;
	
	// 长度不对直接报错
	if (n != 3) return 0;  
	
	// 存储数据
	for (i = 0 ; i<3 ;i++)
	{
		c = s[i];
		if (c >= 'a' && c <= 'z') c = (char)(c - 32);   // 兼容小写 
		if (c != 'L' && c != 'X') return 0;  
		get_data[i] = c;
	}
	return 1;
}

//串口输入指令
uint8_t bonus_input(void)
{
	char    buf[4];          /* 3 个字符 + 结束符，留 1 字节余量 */
	uint8_t n  = 0;
	uint8_t ch;

	UART_SendStr("\r\nINPUT ROUTE (L/X x3, e.g. LXL): \r\n");
	LED_ON();
	(void)huart1.Instance->SR;
	(void)huart1.Instance->DR;
	while(1)
	{
		/* ---------- 只在“真的收到字节”时才处理，其余一律跳过 ---------- */
		if (HAL_UART_Receive(&huart1, &ch, 1, 0) != HAL_OK)
		{
			continue;              /* 没收到：什么都不做，绝不碰 ch */
		} 

		/* 1) 回车 / 换行：结束本次输入，交给校验函数 */
		if (ch == '\r' || ch == '\n')
		{
			if (n == 0)  { UART_SendStr("RETRY: \r\n"); continue; };               /* 空回车，忽略 */

			if (data_analysis(buf, n))
			{
				UART_SendStr("OK\r\n");
				LED_OFF();
				Beep(300);
				return 1;
			}

			/* 非法指令：拒绝并重新等待 */
			UART_SendStr("ERROR\r\n");
			Beep(80); HAL_Delay(80);
			Beep(80); HAL_Delay(80);
			Beep(80);
			n = 0;
			UART_SendStr("RETRY: \r\n");
			continue;
		}

		/* 2) 退格：允许改错（0x08 = Ctrl+H，0x7F = Del） */
		if (ch == 0x08 || ch == 0x7F)
		{
			if (n > 0) n--;
			continue;
		}

		/* 3) 普通字符：最多收 3 个 */
		if (n < 3)
		{
			buf[n++] = (char)ch;
			HAL_UART_Transmit(&huart1, &ch, 1, 10); 
		}
		else
		{
			/* 第 4 个字符出现 → 这条指令一定不合法，直接判错 */
			UART_SendStr("TOO LONG\r\n");
			Beep(80); HAL_Delay(80);
			Beep(80); HAL_Delay(80);
			Beep(80);
			n = 0;
			UART_SendStr("RETRY: \r\n");
		}
	}
}

void problem5(void)
{
	uint8_t i;
	
	for(i = 0;i<3;i++)
	{
		if (get_data[i] == 'L') route_l();
		else                 	route_x();
		
	}
}
	
	
void problem6(void)
{
      // 盲走到看见线 
	drive_stright_stop(0.0f,2,0);
	voice_light();
	
	
	// 线出现后再确认 30 拍，防止不会因为轨迹卡在两个传感器之间直接切到下一个步骤
	turn_start_confirm(0.0f);  
	continue_rota(-140.25f);//143.0f   //直接雷霆大掉头


	// 盲走到看见线 
	drive_stright_stop(-140.85f,3,0);
	voice_light();


	// 循迹过弯：连续 8 拍丢线才算真出弯 
	turn_finished_confirm();
	stop();
	voice_light();
}	
	
//当按下按键后才开始行走，防止来不及拆掉串口模块
void wait_key_start(void)
{
	uint8_t  on = 0;
	uint32_t t  = HAL_GetTick();

	stop();                       // 确保车是停着的 

	while(1)
	{
		if (Key_Scan(20) != 0)    // 非阻塞：这一拍检测到一次按下 
		{
			LED_OFF();
			Beep(120);            // 短响：确认按键收到 
			return;
		}

		// LED 慢闪，表示输入完成，等待按键按下 
		if (HAL_GetTick() - t >= 400U)
		{
			t  = HAL_GetTick();
			on = !on;
			if (on) LED_ON();
			else    LED_OFF();
		}

		HAL_Delay(10);            // 固定 10ms 周期，Key_Scan 的去抖才准 
	}
}


//定时中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		//Rota_Update(-50.2f, GetYaw());
		//Rota_Update(143.0f, GetYaw());
		//walking_blindly(12, 12, 0.0f);
		//SpeedLoop_Update(10 ,10);
		//SpeedLoop_Update(40,40);
		//current_sensor_state = Sensor_GetState();
		//turn_Update(&Turn);//测完记得注释
		
		if (MPU6050_IsOnline()) MPU6050_Proc(); // 如果在线就一直更新
		
	}
}

//自检串口发送+初始化
void selfcheck(void)
{
	/* 先把 PWM/编码器启动好（占空比 0，不会转） */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* 右轮 PA8 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   /* 左轮 PA11 */
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* 左编码器 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* 右编码器 */
	
	
	Sensor_Init();          
	Key_Init();             
	LED_Init();             
	Buzzer_Init();
	Task_Init();
	UART_SendStr("\r\n===== BOOT =====\r\n");

	Sensor_Init();          UART_SendStr("Sensor ok\r\n");
	Key_Init();             UART_SendStr("Key ok\r\n");
	LED_Init();             UART_SendStr("LED ok\r\n");
	Buzzer_Init();          UART_SendStr("Buzzer ok\r\n");
	Task_Init();            UART_SendStr("Task ok\r\n");

	// ===== MPU6050：上电等它稳，再重试探测 =====
	// 冷启动（电池直接上电）时 GY-521 自己的稳压器要 ~100ms 才稳，
	// 一上电就 Ping 会失败；在线调试时板子早通电了，所以偶尔能读到。 
	HAL_Delay(200);
	{
		uint8_t i;
		for (i = 0U; i < 10U; i++)
		{
			MPU6050_Init();
			if (MPU6050_IsOnline()) break;
			UART_SendStr("MPU retry...\r\n");
			HAL_Delay(50);
		}
	}
	
	UART_SendStr(MPU6050_IsOnline() ? "MPU ok\r\n" : "MPU NOT FOUND (check PB8/PB9)\r\n");
	
	// 只有在线才标定，否则 500 次读失败会白等好几秒 
	if (MPU6050_IsOnline())
	{
		MPU6050_Calibrate(500);   /* 2s，车必须完全不动 */
		MPU6050_ResetYaw();
	}

	/* ===== 最后才使能驱动 + 开 10ms 中断 ===== */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // STBY = 高 
	HAL_TIM_Base_Start_IT(&htim3);
	
}

void Sensor_Show(void)
{
    uint8_t raw = Sensor_ReadRaw();      /* bit6=最左 … bit0=最右 */
    char    buf[10];
    int     i;

    for (i = 0; i < 7; i++)
        buf[i] = ((raw >> (6 - i)) & 1u) ? '#' : '.';   /* # 判黑  . 判白 */

    buf[7] = '\r'; buf[8] = '\n'; buf[9] = '\0';
    UART_SendStr(" S=");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, 9, 100);
}

//状态机
void problem_loop(void)
{
	 // 等待按键按下
	uint8_t step = 0;
	
	// 上电先静止
	stop();  
	LED_ON();   
	
	while (1)
	{
		// 待机，等待按钮按下
		if (Key_Scan(20) != 0)
		{
			// 等待手松开
			while (Key_IsDown()) HAL_Delay(10);
			//按键消抖，确保真的松开
			HAL_Delay(50);      
			// 切换下一题
			step++;
			
			LED_OFF();   
			stop(); 
			
			switch (step)
			{
				case 1: problem1(); break;
				case 2: mpu_recalibrate(250) ; problem2(); break;
				case 3: mpu_recalibrate(250) ; problem3(); break;
				case 4: mpu_recalibrate(500) ; problem4(); break;
				case 5: //按两下
						while(Key_Scan(20) != 1)
						{
							HAL_Delay(10);
						}
						bonus_input(); 
						wait_key_start();
						mpu_recalibrate(500) ; 
						problem5(); 
						break;
				case 6: mpu_recalibrate(500) ; problem6(); break;
				
				default: break;
			}
			//统一完成声光提示一次
			stop(); 
			voice_light();
		}
		HAL_Delay(10);
	}
	
	
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	//自检+初始化
	selfcheck();
	
	//测试代码
	//problem1();
	//problem2();
	//problem3();
	//problem4();
	/*
	while(Key_Scan(20) != 1)
	{
		HAL_Delay(10);
	}
	bonus_input(); 
	wait_key_start();
	mpu_recalibrate(500) ; 
	problem5(); 
	stop(); 
	voice_light();
	
		
	wait_key_start();
	mpu_recalibrate(500) ; 
	problem6(); 
	stop(); 
	voice_light();
	*/
	//problem6(); 
	problem_loop();   

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
	while(1)
	{
		Car_State_Print();
		//Sensor_Show();
	}

	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |STBY_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED1_Pin RED2_Pin RED3_Pin RED4_Pin
                           RED5_Pin RED6_Pin */
  GPIO_InitStruct.Pin = RED1_Pin|RED2_Pin|RED3_Pin|RED4_Pin
                          |RED5_Pin|RED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED7_Pin RED8_Pin */
  GPIO_InitStruct.Pin = RED7_Pin|RED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin BIN2_Pin
                           STBY_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |STBY_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
