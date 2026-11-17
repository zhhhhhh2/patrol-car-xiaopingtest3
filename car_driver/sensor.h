/**
  ******************************************************************************
  * @file    sensor.h
  * @brief   灰度/红外循迹传感器驱动（HAL 版）
  *
  * 来源：队友 A 的标准库 Sensor.c / Sensor.h，已改成 HAL，引脚改用 CubeMX
  *       工程里的 RED1~RED8 宏，不再出现任何标准库函数。
  *
  * 硬件约定（和 main.h 里 CubeMX 生成的宏一致）：
  *   arr[0] 最左 = RED7(PB0)   arr[1] = RED6(PA7)   arr[2] = RED5(PA6)
  *   arr[3] 中间 = RED4(PA5)   arr[4] = RED3(PA4)                                  arr[5] = RED2(PA3)
  *   arr[6] 最右 = RED1(PA2)
  *   RED8(PB1) 目前没用（模块只有 7 路输出），留着以后换 8 路模块直接接上。
  ******************************************************************************
  */
#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 用到的传感器路数（模块 7 路输出） */
#define SENSOR_NUM              7

/* 电平极性
 *   1 = 检测到黑线时输出高电平（队友模块 + 上拉的情况）
 *   0 = 相反（白高黑低），代码会自动取反，不用改逻辑
 */

/* 丢线时 Sensor_Find() 的应急输出（原地向右找线） */
#define SENSOR_LOST_PWM_L       400
#define SENSOR_LOST_PWM_R       0

/* 传感器状态机返回值 */
typedef enum
{
    SENSOR_STATE_NORMAL = 0,        /* 正常巡线：至少一路压到黑线 */
    SENSOR_STATE_LOST               /* 全白：丢线 / 盲走阶段 */
} SensorState_t;

/* ---------------- 接口（函数名和队友原来的完全一致） ---------------- */

void          Sensor_Init(void);                              /* 初始化 7 路输入（上拉） */
uint8_t       Sensor_ReadRaw(void);                           /* bit6..bit0 = 最左..最右 */
void          Sensor_ReadArray(uint8_t *arr7);                /* arr7[0] 最左 ... arr7[6] 最右 */
float         Sensor_GetError(void);                          /* 加权偏差 [-1.0, 1.0] */
SensorState_t Sensor_GetState(void);                          /* NORMAL / LOST */
void          Sensor_Find(uint16_t *pwm_L, uint16_t *pwm_R);    /* 查表循迹，输出 0~1000 */

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H */
