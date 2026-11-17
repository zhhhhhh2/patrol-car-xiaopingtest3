/**
  * @file    mi2c.h
  * @brief   软件 I2C（GPIO 位操作）—— 队友标准库版本改成 HAL 版
  * @note    原文件：C:\Users\zh\Desktop\11-1MPU - 副本\Hardware\MI2C.c/.h
  *
  *          注意：队友原来把 SCL 放在 PB10、SDA 放在 PB11，
  *          但本工程 PB10/PB11 是电机方向脚 AIN1/AIN2，绝对不能占用！
  *          所以这里默认改成 I2C1 重映射后的 PB8/PB9（和硬件 I2C 同一对脚，
  *          你板子上陀螺仪接哪两个脚就改下面两个宏）。
  *
  *          默认走硬件 I2C（见 mpu6050.h 里的 MPU6050_USE_HW_I2C），
  *          这个软件 I2C 只是备胎：把 MPU6050_USE_HW_I2C 改成 0 就会用它。
  */
#ifndef __MI2C_H
#define __MI2C_H

#include "main.h"
#include "stm32f1xx_hal.h"

/* ---------------- 软件 I2C 引脚（换脚只改这里） ---------------- */
#ifndef MI2C_SCL_GPIO_PORT
#define MI2C_SCL_GPIO_PORT   GPIOB
#define MI2C_SCL_GPIO_PIN    GPIO_PIN_8
#endif

#ifndef MI2C_SDA_GPIO_PORT
#define MI2C_SDA_GPIO_PORT   GPIOB
#define MI2C_SDA_GPIO_PIN    GPIO_PIN_9
#endif

/* 每个电平的保持时间：2us ≈ 250kHz（MPU6050 上限 400kHz）。
   队友原来用 10us（≈50kHz），想稳妥/线长就改回 10。 */
#ifndef MI2C_DELAY_US
#define MI2C_DELAY_US        2U
#endif

/* 读 SDA 的方式：
   0 = 直接读开漏输出脚的电平（队友原做法，要求 SDA 上有上拉电阻，
       GY-521 模块自带 4.7k 上拉，所以默认就行）
   1 = 读之前临时切成"输入+内部上拉"，读完再切回开漏（没有外部上拉时用这个） */
#ifndef MI2C_SDA_READ_WITH_PULLUP
#define MI2C_SDA_READ_WITH_PULLUP   0
#endif

void    MI2C_Init(void);
void    MI2C_Start(void);
void    MI2C_Stop(void);
void    MI2C_SendByte(uint8_t Byte);
uint8_t MI2C_ReceiveByte(void);
void    MI2C_SendAck(uint8_t AckBit);
uint8_t MI2C_ReceiveAck(void);
void    MI2C_W_SCL(uint8_t BitValue);
void    MI2C_W_SDA(uint8_t BitValue);
uint8_t MI2C_R_SDA(void);

#endif
