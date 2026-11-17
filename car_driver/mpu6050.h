/**
  * @file    mpu6050.h
  * @brief   MPU6050 六轴姿态：寄存器读写 + 互补滤波解算 yaw/pitch/roll
  * @note    原文件：C:\Users\zh\Desktop\11-1MPU - 副本\Hardware\MPU6050.h/.c（标准库）
  *          这里改成 HAL 版，函数名全部保留，队友那边调用 MPU6050_Proc() / MPU6050_YawData()
  *          的代码一行都不用改。
  *
  *          两处必须注意的工程差异：
  *          1) 队友用 TIM3 做 10ms 节拍（自己写了 TIM3_IRQHandler）。本工程 TIM3 已被
  *             CubeMX 配成 10ms 并在 stm32f1xx_it.c 里接管，不能再定义同名中断
  *             （否则链接重复定义）。所以这里把节拍改成 HAL_GetTick() 判断，
  *             MPU6050_TickFlag() 依旧是"每 10ms 返回一次 1"，接口不变。
  *          2) 队友软件 I2C 用 PB10/PB11，本工程这两个脚是 AIN1/AIN2 电机方向脚，
  *             所以默认改走已配好的硬件 I2C1（PB8/PB9，100kHz）。
  *             硬件走不通就把 MPU6050_USE_HW_I2C 改成 0，自动改用 mi2c.c 的软件 I2C。
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "stm32f1xx_hal.h"

/* 1 = 用硬件 I2C1（hi2c1，PB8/PB9）；0 = 用 mi2c.c 的软件 I2C */
#ifndef MPU6050_USE_HW_I2C
#define MPU6050_USE_HW_I2C      1
#endif

/* 从机地址：0x68（7 位），标准库里的 0xD0 就是它左移一位 */
#define MPU6050_ADDR_7BIT       0x68U
#define MPU6050_ADDR_8BIT       (MPU6050_ADDR_7BIT << 1)    /* 0xD0 */
/* AD0 脚悬空/接高时地址变成 0x69，Init 会自动两个都试一遍 */
#define MPU6050_ADDR_7BIT_ALT   0x69U

/* Proc 的建议调用周期（ms）。内部 dt 由 DWT 实测，所以偶尔晚一点也不影响角度 */
#define MPU6050_TICK_MS         10U
/* 单次 I2C 超时（ms）。读 14 字节 @100kHz 约 1.5ms，给 10ms 足够 */
#define MPU6050_I2C_TIMEOUT     10U

/* ---------------- 安装姿态无关的 yaw（重点） ----------------
 * 1 = 把角速度投影到"加速度计测到的重力方向"上再积分（推荐，默认）。
 *     不管 MPU 是水平放着、竖着贴、斜着绑、还是杜邦线吊着，车原地转时
 *     转轴都是世界竖直方向，而竖直方向正好就是静止时加速度计测到的那个方向，
 *     所以 yaw_rate = (wx,wy,wz) · (ax,ay,az)/|a| 就能拿到真正的绕竖直角速度。
 * 0 = 老算法：用 roll/pitch 凑 sr/cr/cp 再除以 cos(pitch)。
 *     这个要求模块基本水平安装，而且 pitch 接近 ±90° 时 1/cp 会爆掉。
 */
#ifndef MPU6050_TILT_FREE_YAW
#define MPU6050_TILT_FREE_YAW   1
#endif

/* 重力方向的低通系数（0.02 ≈ 在 100Hz 下 0.5s 时间常数），滤车体振动 */
#define MPU6050_GRAV_LPF        0.05f

/* ---------------- 初始化 ---------------- */
void    MPU6050_Init(void);          /* 唤醒 + 配置量程 + 使能 DWT */
void    MPU6050_Calibrate(uint16_t number);     /* 静止零偏标定（500 次 × 4ms ≈ 2s，车必须不动） */
uint8_t MPU6050_IsOnline(void);      /* 1 = 总线上读到了器件（WHO_AM_I 非 0/0xFF 也算通） */

/* ---------------- 寄存器读写（保留队友接口） ---------------- */
void    MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);   /* 失败返回 0xFF（不是 0x00） */

/* ---------------- I2C 排障 ---------------- */
uint8_t  MPU6050_ProbeAddr(uint8_t addr7);  /* 1 = 该 7 位地址有应答（试 3 次） */
uint8_t  MPU6050_Ping(void);                /* 重新探测 0x68/0x69，刷新 s_online */
uint8_t  MPU6050_ScanBus(uint8_t *Found, uint8_t MaxFound);  /* 扫 0x08~0x77，返回有应答的个数 */
uint8_t  MPU6050_WhoAmIRaw(void);           /* 最近一次读到的 WHO_AM_I（正常 0x68） */
uint8_t  MPU6050_GetAddr7(void);            /* 当前实际在用的 7 位地址 */
uint32_t MPU6050_ErrCount(void);            /* 累计 I2C 失败次数，一直在涨就是没通 */

/* ---------------- 采数据 / 解算 ---------------- */
void    MPU6050_GetData(void);       /* 读 14 字节并换算成物理量（不积分） */
void    MPU6050_Proc(void);          /* 读一次 + 互补滤波积分；建议 10ms 调一次 */
uint8_t MPU6050_TickFlag(void);      /* 每 10ms 返回一次 1，配合 Proc 用 */

/* ---------------- 取结果 ---------------- */
float   MPU6050_YawData(void);       /* -180 ~ +180 度，转向就靠它 */
float   MPU6050_PitchData(void);
float   MPU6050_RollData(void);
float   MPU6050_YawRateData(void);   /* 度/秒 */
float   MPU6050_MountTiltDeg(void);  /* 模块安装倾斜角（度），仅自检用；0=平放 90=竖贴 */

float   MPU6050_AXData(void);
float   MPU6050_AYData(void);
float   MPU6050_AZData(void);
float   MPU6050_GXData(void);
float   MPU6050_GYData(void);
float   MPU6050_GZData(void);
float   MPU6050_TempData(void);

float   MPU6050_BiasXData(void);
float   MPU6050_BiasYData(void);
float   MPU6050_BiasZData(void);

void    MPU6050_ResetYaw(void);      /* yaw 清零（每次发车前调一次） */
void    MPU6050_SetYaw(float deg);   /* 把 yaw 直接设成某个角度 */

/* ---------------- 调试镜像（在 mpu6050.c 里定义，只写不读，不会被优化掉） ----------------
 * Keil Debug → Watch 1 里直接加变量名就能看，不需要串口：
 *   g_dbg_online   1 = I2C 通了；0 = 陀螺仪没接上
 *   g_dbg_yaw      航向角（度）
 *   g_dbg_yaw_rate 绕竖直轴的角速度（度/秒），静止应≈0，转起来应是几十~几百
 *   g_dbg_tilt     安装倾斜角（度），非 0 说明投影补偿在工作
 */
extern volatile uint8_t g_dbg_online;
extern volatile float   g_dbg_yaw;
extern volatile float   g_dbg_yaw_rate;
extern volatile float   g_dbg_tilt;

/* ---------------- 定时相关（HAL 版已不再占 TIM3） ---------------- */
void    MPU6050_TimeBaseInit(void);  /* 空函数，只为兼容队友代码；节拍见 MPU6050_TickFlag() */
void    MPU6050_DwtInit(void);       /* 使能 DWT 周期计数器（Init 里已自动调用） */
uint32_t MPU6050_GetUs(void);        /* 微秒时间戳 */

#endif
