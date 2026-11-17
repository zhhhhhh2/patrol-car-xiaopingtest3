/**
  ******************************************************************************
  * @file    sensor.c
  * @brief   灰度/红外循迹传感器驱动（HAL 版）
  *
  * 标准库 → HAL 的对应关系（改了这几处，逻辑一行没动）：
  *   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|GPIOB, ENABLE)
  *       → 不需要了：CubeMX 的 MX_GPIO_Init() 里已经使能 GPIOA/GPIOB 时钟
  *   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU
  *       → GPIO_InitStruct.Mode = GPIO_MODE_INPUT; Pull = GPIO_PULLUP
  *   GPIO_Init(GPIOA, &GPIO_InitStructure)
  *       → HAL_GPIO_Init(GPIOA, &GPIO_InitStruct)
  *   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)
  *       → HAL_GPIO_ReadPin(RED1_GPIO_Port, RED1_Pin)
  ******************************************************************************
  */
#include "sensor.h"
#include "main.h"                   /* RED1_xxx ~ RED8_xxx 引脚宏（CubeMX 生成） */
#include "stm32f1xx_hal.h"

#define turn_base_speed (500)

/* 引脚表：下标 = arr 下标，0 最左，6 最右
 * 注意：队友原来的顺序是 OUT7(最左)=PB0 … OUT1(最右)=PA2，这里保持电气顺序不变。
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} SensorPin_t;

static const SensorPin_t s_pin[SENSOR_NUM] =
{
    {RED7_GPIO_Port, RED7_Pin},     /* arr[0] 最左  PB0  (=队友的 OUT7) */
    {RED6_GPIO_Port, RED6_Pin},     /* arr[1]       PA7  (OUT6) */
    {RED5_GPIO_Port, RED5_Pin},     /* arr[2]       PA6  (OUT5) */
    {RED4_GPIO_Port, RED4_Pin},     /* arr[3] 中间  PA5  (OUT4) */
    {RED3_GPIO_Port, RED3_Pin},     /* arr[4]       PA4  (OUT3) */
    {RED2_GPIO_Port, RED2_Pin},     /* arr[5]       PA3  (OUT2) */
    {RED1_GPIO_Port, RED1_Pin},     /* arr[6] 最右  PA2  (OUT1) */
};

/* 加权平均用的权重：最左 -3，中间 0，最右 +3 */
static const int8_t weight[SENSOR_NUM] = {-3, -2, -1, 0, 1, 2, 3};

/* 读一个脚，并且把极性统一成“1 = 压到黑线” */
static uint8_t Sensor_PinRead(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t v = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;

    /* 当前模块：压到黑线输出高电平。若换成极性相反的模块，把这行取消注释 */
    /* v = (uint8_t)(1u - v); */

    return v;
}

/**
  * @brief  初始化 7 路输入
  * @note   CubeMX 里 RED1~RED8 已经配成 GPIO_INPUT / NOPULL 了，
  *         这里把真正用到的 7 路再配一次"上拉输入"：
  *         模块掉电或线断了就读到高电平（=白），不会乱跳触发误判。
  */
void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t i;

    for (i = 0; i < SENSOR_NUM; i++)
    {
        GPIO_InitStruct.Pin   = s_pin[i].pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(s_pin[i].port, &GPIO_InitStruct);
    }
}

/**
  * @brief  读 7 路，拼成一个字节：bit6 = 最左 … bit0 = 最右（bit7 恒 0）
  */
uint8_t Sensor_ReadRaw(void)
{
    uint8_t s[SENSOR_NUM];
    uint8_t raw = 0;
    uint8_t i;

    Sensor_ReadArray(s);

    for (i = 0; i < SENSOR_NUM; i++)
    {
        if (s[i] != 0u)
        {
            raw |= (uint8_t)(1u << (SENSOR_NUM - 1u - i));   /* 最左 → bit6 */
        }
    }

    return raw;
}

/**
  * @brief  读 7 路到数组，arr7[0] 最左 … arr7[6] 最右
  */
void Sensor_ReadArray(uint8_t *arr7)
{
    uint8_t i;

    for (i = 0; i < SENSOR_NUM; i++)
    {
        arr7[i] = Sensor_PinRead(s_pin[i].port, s_pin[i].pin);
    }
}

/**
  * @brief  加权平均法算位置偏差，返回值范围 [-1.0, 1.0]：负数偏左，正数偏右
  * @note   除以 (压线条数 × 3) 可以让压线越多、偏差越小，循迹更平滑
  */
float Sensor_GetError(void)
{
    uint8_t s[SENSOR_NUM];
    int sum_weight = 0;
    int active_cnt = 0;
    int i;

    Sensor_ReadArray(s);

    for (i = 0; i < SENSOR_NUM; i++)
    {
        if (s[i] != 0u)
        {
            sum_weight += weight[i];
            active_cnt++;
        }
    }

    if (active_cnt == 0)
    {
        return 0.0f;                /* 丢线返回 0 */
    }

    return (float)sum_weight / (float)(active_cnt * 3);
}

/**
  * @brief  当前循迹状态：全白 = LOST，否则 NORMAL
  */
SensorState_t Sensor_GetState(void)
{
    uint8_t s[SENSOR_NUM];
    uint8_t cnt = 0;
    uint8_t i;

    Sensor_ReadArray(s);

    for (i = 0; i < SENSOR_NUM; i++)
    {
        cnt += s[i];
    }

    if (cnt == 0u)
    {
        return SENSOR_STATE_LOST;   /* 全白：丢线 / 盲走阶段 */
    }

    return SENSOR_STATE_NORMAL;     /* 正常巡线 */
}

/**
  * @brief  查表法循迹：根据误差直接给出左右轮 PWM（0 ~ 1000）
  * @param  base_speed 基础速度
  * @param  pwm_L      输出：左轮 PWM
  * @param  pwm_R      输出：右轮 PWM
  */
void Sensor_Find(uint16_t *pwm_L, uint16_t *pwm_R)
{
    float   error;
    int16_t left;
    int16_t right;

    /* 1. 丢线处理：原地向右找线（PWM 不能为负，用 0 和正数做差速） */
    if (Sensor_GetState() == SENSOR_STATE_LOST)
    {
        *pwm_L = SENSOR_LOST_PWM_L;
        *pwm_R = SENSOR_LOST_PWM_R;
        return;
    }

    /* 2. 误差放大到 -100 ~ 100 */
    error = Sensor_GetError() * 100.0f;

    left  = turn_base_speed;
    right = turn_base_speed;

    /* 3. 分段差速 */
    if (error <= -95.0f)            /* 极限偏左：右打满 */
    {
        left  += 400;
        right -= 400;
    }
    else if (error <= -80.0f)       /* 严重偏左 */
    {
        left  += 250;
        right -= 250;
    }
    else if (error <= -45.0f)       /* 中等偏左 */
    {
        left  += 150;
        right -= 100;
    }
    else if (error <= -15.0f)       /* 轻微偏左 */
    {
        left  += 60;
        right -= 40;
    }
    else if (error >= 95.0f)        /* 极限偏右：左打满 */
    {
        left  -= 400;
        right += 400;
    }
    else if (error >= 80.0f)        /* 严重偏右 */
    {
        left  -= 250;
        right += 250;
    }
    else if (error >= 45.0f)        /* 中等偏右 */
    {
        left  -= 100;
        right += 150;
    }
    else if (error >= 15.0f)        /* 轻微偏右 */
    {
        left  -= 40;
        right += 60;
    }
    else                            /* 走正中 */
    {
        left  = turn_base_speed;
        right = turn_base_speed;
    }

    /* 4. 限幅 0 ~ 1000 */
    if (left  > 1000) left  = 1000;
    if (left  < 0)    left  = 0;
    if (right > 1000) right = 1000;
    if (right < 0)    right = 0;

    *pwm_L = left;
    *pwm_R = right;
}
