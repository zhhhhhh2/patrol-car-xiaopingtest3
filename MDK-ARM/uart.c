#include "team.h"
#include "main.h" 
#include "stm32f1xx_hal.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

//发送角度
HAL_StatusTypeDef UART_SendUint8(UART_HandleTypeDef *huart, uint8_t data)
{
    return HAL_UART_Transmit(huart, &data, 1, 100);
}

//串口发送调试
void UART_SendStr(const char *s)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}

// 发送 int16_t 整数，支持正负 
void UART_SendInt(int16_t num)
{
    char buf[8];
    uint8_t i = 0;
    int16_t val = num;

    if(val < 0)
    {
        buf[i++] = '-';
        val = -val;
    }

    // 数字逆序存入临时数组
    char tmp[6];
    uint8_t t = 0;
    if(val == 0)
    {
        tmp[t++] = '0';
    }
    else
    {
        while(val > 0)
        {
            tmp[t++] = '0' + (val % 10);
            val /= 10;
        }
    }
    // 反转
    while(t > 0)
    {
        buf[i++] = tmp[--t];
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, i, 100);
}

