/**
  ******************************************************************************
  * @file    key.c
  * @brief   按键驱动（HAL 版）
  *
  * 标准库 → HAL：
  *   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
  *       → 不需要：MX_GPIO_Init() 已经使能时钟
  *   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU
  *       → Mode = GPIO_MODE_INPUT, Pull = GPIO_PULLUP
  *   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0
  *       → HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET
  *   Delay_ms(20)
  *       → HAL_Delay(20)（HAL_Delay 基于 SysTick，1ms 单位）
  ******************************************************************************
  */
#include "key.h"
#include "main.h"                   /* KEY_Pin / KEY_GPIO_Port */
#include "stm32f1xx_hal.h"

/**
  * @brief  按键初始化：上拉输入
  * @note   CubeMX 里 KEY 配的是 GPIO_MODE_INPUT + NOPULL，
  *         这里补成上拉，省掉外部上拉电阻，松开时一定是高电平。
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = KEY_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  读按键电平，统一成 1 = 按着
  */
uint8_t Key_IsDown(void)
{
    GPIO_PinState st = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

    return (st == GPIO_PIN_RESET) ? 1u : 0u;   /* 按键低有效：按下是低电平 */
}

/**
  * @brief  阻塞版按键扫描（和队友原来的 Key_GetNum 行为一致）
  * @retval 1 = 有一次完整的"按下并松开"
  * @note   内部会一直等松手，所以只能放在主循环里，不能放中断。
  */
uint8_t Key_GetNum(void)
{
    if (Key_IsDown() == 0u)
    {
        return 0u;                  /* 没按 */
    }

    HAL_Delay(KEY_DEBOUNCE_MS);     /* 消除按下抖动 */

    if (Key_IsDown() == 0u)
    {
        return 0u;                  /* 是抖了一下，不算 */
    }

    while (Key_IsDown() != 0u)      /* 等松手，防止一次按下被算成多次 */
    {
        /* 空循环 */
    }

    HAL_Delay(KEY_DEBOUNCE_MS);     /* 消除松开抖动 */

    return 1u;
}

/**
  * @brief  非阻塞版按键扫描：检测到"按下"的瞬间返回 1
  * @param  debounce_ms 两次有效跳变之间的最小间隔，传 KEY_DEBOUNCE_MS 即可
  * @note   必须在固定周期里调用（比如 10ms 一次），否则计时不准
  */
uint8_t Key_Scan(uint16_t debounce_ms)
{
    static uint8_t  last_st   = 0;
    static uint32_t last_tick = 0;

    uint8_t  now_st = Key_IsDown();
    uint32_t now    = HAL_GetTick();

    if (now_st == last_st)
    {
        return 0u;                  /* 电平没变 */
    }

    if ((now - last_tick) < (uint32_t)debounce_ms)
    {
        return 0u;                  /* 抖动，忽略 */
    }

    last_tick = now;
    last_st   = now_st;

    return (now_st != 0u) ? 1u : 0u;    /* 只在"按下"这一刻返回 1 */
}
