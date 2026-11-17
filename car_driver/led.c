/**
  ******************************************************************************
  * @file    led.c
  * @brief   LED + 蜂鸣器声光提示（HAL 版）
  *
  * 标准库 → HAL：
  *   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|GPIOB, ENABLE) → 不需要（MX_GPIO_Init 已开）
  *   GPIO_Mode_Out_PP + GPIO_Init()        → HAL_GPIO_Init(&GPIO_InitStruct)
  *   GPIO_ResetBits(GPIOA, GPIO_Pin_1)     → HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
  *   GPIO_SetBits(GPIOA, GPIO_Pin_1)       → HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
  *   Delay_ms(100)                         → HAL_Delay(100)
  ******************************************************************************
  */
#include "led.h"
#include "main.h"                   /* LED_Pin / Buzzer_Pin 等 CubeMX 宏 */
#include "stm32f1xx_hal.h"

/* ---------------- LED ---------------- */

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    LED_OFF();                      /* 上电默认灭 */
}

void LED_ON(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   /* PC13 低电平点亮 */
}

void LED_OFF(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);     /* 灭：输出高 */
}

void LED_Turn(void)
{
    /* 读输出寄存器再取反：HAL_GPIO_TogglePin 内部就是这么做的 */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* ---------------- 蜂鸣器 ---------------- */

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = Buzzer_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

    Buzzer_OFF();                   /* 上电默认不响 */
}

void Buzzer_ON(void)
{
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);  /* 蜂鸣器低电平响 */
}

void Buzzer_OFF(void)
{
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET); /* 停：输出低 */
}

void Beep(uint16_t ms)
{
    Buzzer_ON();
    HAL_Delay(ms);                  /* 阻塞，循迹中别用 */
    Buzzer_OFF();
}
