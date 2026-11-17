/**
  ******************************************************************************
  * @file    key.h
  * @brief   按键驱动（HAL 版）
  *
  * 来源：队友 A 的标准库 key.c / key.h。
  * 原代码里有两只按键（PB1 / PB15），统一引脚表里只有一只启动键，
  * 而且 PB1 已经给 RED8（红外第 8 路）用了，所以这里只保留一只键。
  *
  * !! 引脚以 CubeMX 的 main.h 为准：KEY_Pin = PC15 !!
  *    引脚表里写的是 PB15，工程里配的是 PC15，两者不一致。
  *    确认实际焊的是哪个之后，只需要改 main.h 里的
  *      #define KEY_Pin / #define KEY_GPIO_Port
  *    本文件一行都不用动。
  ******************************************************************************
  */
#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

 #ifdef __cplusplus
extern "C" 
{
#endif

/* 消抖时间（ms） */
#define KEY_DEBOUNCE_MS     20

/* 按下时的电平：0 = 按下为低电平（上拉输入，板子上基本都是这种） */

void    Key_Init(void);                 /* 配置成上拉输入 */
uint8_t Key_IsDown(void);               /* 1 = 当前按着，0 = 松开 */

/* 阻塞版，和队友原来的用法一样：按下并松开后返回 1，否则返回 0
   注意：里面有 while 等松手，不要放进定时器中断里调。 */
uint8_t Key_GetNum(void);

/* 非阻塞版：状态机里用这个。每次调用做一次边沿检测，
   检测到"按下"返回 1。建议每 10ms 调一次（Task_Update 里已经这么做了）。 */
uint8_t Key_Scan(uint16_t debounce_ms);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */
