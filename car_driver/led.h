/**
  ******************************************************************************
  * @file    led.h
  * @brief   LED + 蜂鸣器声光提示（HAL 版）
  *
  * 来源：队友 A 的标准库 LED.c / LED.h（原来用 PA1/PA2 两只 LED）。
  * 我们的主控统一引脚表只有：
  *   LED     = PC13（板载蓝灯，低电平点亮）
  *   Buzzer  = PB5 （高电平响）
  *
  * 所以把"LED1/LED2"两只灯合并成 LED_ON/LED_OFF/LED_Turn，
  * 另外补上蜂鸣器（队友工程里没有蜂鸣器驱动，我们自己加的）。
  ******************************************************************************
  */
#ifndef __LED_H
#define __LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 电平极性：1 = 低电平点亮（PC13 板载灯就是这样）；0 = 高电平点亮 */

/* 蜂鸣器极性：1 = 高电平响（PB5 接无源/有源模块的 IO 一般都是这种） */

/* 常用提示音时长 */
#define BEEP_SHORT_MS           80
#define BEEP_LONG_MS            300

/* ---------------- LED ---------------- */
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Turn(void);            /* 翻转，做闪灯用 */

/* ---------------- 蜂鸣器 ---------------- */
void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);

/* 阻塞版响 ms 毫秒（队友风格，简单粗暴）
   注意：循迹过程中不要用，请用状态机里的非阻塞 Task_Beep()。 */
void Beep(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
