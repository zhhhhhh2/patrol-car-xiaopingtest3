/**
  * @file    mpu6050_reg.h
  * @brief   MPU6050 寄存器地址表
  * @note    原文件是队友标准库工程里的 Hardware/MPU6050_Reg.h，
  *          这里只改了文件名（统一小写），内容一个字节没动。
  */
#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

#define  MPU6050_SMPLRT_DIV     0x19    /* 采样率分频 */
#define  MPU6050_CONFIG         0x1A    /* 低通滤波 */
#define  MPU6050_GYRO_CONFIG    0x1B    /* 陀螺仪量程 */
#define  MPU6050_ACCEL_CONFIG   0x1C    /* 加速度量程 */

#define  MPU6050_ACCEL_XOUT_H   0x3B
#define  MPU6050_ACCEL_XOUT_L   0x3C
#define  MPU6050_ACCEL_YOUT_H   0x3D
#define  MPU6050_ACCEL_YOUT_L   0x3E
#define  MPU6050_ACCEL_ZOUT_H   0x3F
#define  MPU6050_ACCEL_ZOUT_L   0x40
#define  MPU6050_TEMP_OUT_H     0x41
#define  MPU6050_TEMP_OUT_L     0x42
#define  MPU6050_GYRO_XOUT_H    0x43
#define  MPU6050_GYRO_XOUT_L    0x44
#define  MPU6050_GYRO_YOUT_H    0x45
#define  MPU6050_GYRO_YOUT_L    0x46
#define  MPU6050_GYRO_ZOUT_H    0x47
#define  MPU6050_GYRO_ZOUT_L    0x48

#define  MPU6050_PWR_MGMT_1     0x6B    /* 电源管理 1 */
#define  MPU6050_PWR_MGMT_2     0x6C
#define  MPU6050_WHO_AM_I       0x75    /* 器件 ID，读出来应该是 0x68 */

#endif
