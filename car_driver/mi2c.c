/**
  * @file    mi2c.c
  * @brief   软件 I2C（GPIO 位操作）—— 队友标准库版本改成 HAL 版
  *
  * 标准库 → HAL 对照（逐行改动说明）：
  *   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
  *                          → __HAL_RCC_GPIOB_CLK_ENABLE()
  *   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD
  *                          → g.Mode = GPIO_MODE_OUTPUT_OD        （开漏输出，完全对应）
  *   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz
  *                          → g.Speed = GPIO_SPEED_FREQ_HIGH
  *   GPIO_Init(GPIOB, &GPIO_InitStructure)
  *                          → HAL_GPIO_Init(GPIOB, &g)
  *   GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)v)
  *                          → HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, v ? GPIO_PIN_SET : GPIO_PIN_RESET)
  *   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)
  *                          → HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET
  *   GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11)
  *                          → HAL_GPIO_WritePin(..., GPIO_PIN_SET) 两次
  *   Delay_us(10)           → MI2C_DelayUs(MI2C_DELAY_US)（用 DWT 内核周期计数器，比空循环准）
  */
#include "mi2c.h"

/* ===================== 微秒延时（DWT 周期计数器） ===================== */
static uint8_t s_dwt_ready = 0U;

static void MI2C_DwtEnable(void)
{
    if (s_dwt_ready != 0U)
    {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 打开跟踪单元 */
    DWT->CYCCNT = 0U;                                 /* 清计数器 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动计数 */
    s_dwt_ready = 1U;
}

static void MI2C_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    MI2C_DwtEnable();
    ticks = us * (SystemCoreClock / 1000000U);        /* 72MHz → 1us = 72 个周期 */
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks)              /* 32 位无符号相减，溢出自动正确 */
    {
    }
}

/* ===================== 引脚操作 ===================== */
void MI2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(MI2C_SCL_GPIO_PORT, MI2C_SCL_GPIO_PIN,
                      (BitValue != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    MI2C_DelayUs(MI2C_DELAY_US);
}

void MI2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(MI2C_SDA_GPIO_PORT, MI2C_SDA_GPIO_PIN,
                      (BitValue != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    MI2C_DelayUs(MI2C_DELAY_US);
}

uint8_t MI2C_R_SDA(void)
{
    uint8_t BitValue;

#if (MI2C_SDA_READ_WITH_PULLUP != 0)
    {
        GPIO_InitTypeDef g = {0};
        /* 切成输入+内部上拉读一次，读完切回开漏输出 */
        g.Pin   = MI2C_SDA_GPIO_PIN;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_PULLUP;
        HAL_GPIO_Init(MI2C_SDA_GPIO_PORT, &g);

        BitValue = (HAL_GPIO_ReadPin(MI2C_SDA_GPIO_PORT, MI2C_SDA_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;

        g.Mode  = GPIO_MODE_OUTPUT_OD;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(MI2C_SDA_GPIO_PORT, &g);
    }
#else
    /* 开漏输出时引脚悬空=高，从机拉低=低，直接读 IDR 就是线上电平 */
    BitValue = (HAL_GPIO_ReadPin(MI2C_SDA_GPIO_PORT, MI2C_SDA_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
#endif
    MI2C_DelayUs(MI2C_DELAY_US);
    return BitValue;
}

/* ===================== 初始化 ===================== */
void MI2C_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏输出：SDA 要被从机拉低产生应答 */
    g.Pull  = GPIO_NOPULL;              /* F1 输出模式下 Pull 无效，上拉靠模块上的 4.7k */
    g.Speed = GPIO_SPEED_FREQ_HIGH;

    g.Pin = MI2C_SCL_GPIO_PIN;
    HAL_GPIO_Init(MI2C_SCL_GPIO_PORT, &g);

    g.Pin = MI2C_SDA_GPIO_PIN;
    HAL_GPIO_Init(MI2C_SDA_GPIO_PORT, &g);

    /* 空闲时两根线都放高 */
    HAL_GPIO_WritePin(MI2C_SCL_GPIO_PORT, MI2C_SCL_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MI2C_SDA_GPIO_PORT, MI2C_SDA_GPIO_PIN, GPIO_PIN_SET);

    MI2C_DwtEnable();
}

/* ===================== 时序（和队友原代码一模一样） ===================== */
void MI2C_Start(void)
{
    MI2C_W_SDA(1);
    MI2C_W_SCL(1);
    MI2C_W_SDA(0);
    MI2C_W_SCL(0);      /* SCL 高时 SDA 下降沿 = 起始 */
}

void MI2C_Stop(void)
{
    MI2C_W_SDA(0);
    MI2C_W_SCL(1);
    MI2C_W_SDA(1);      /* SCL 高时 SDA 上升沿 = 停止 */
}

void MI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0U; i < 8U; i++)
    {
        MI2C_W_SDA((Byte & (0x80U >> i)) ? 1U : 0U);   /* 高位先行 */
        MI2C_W_SCL(1);
        MI2C_W_SCL(0);
    }
}

uint8_t MI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00U;

    MI2C_W_SDA(1);                                     /* 主机释放 SDA */
    for (i = 0U; i < 8U; i++)
    {
        MI2C_W_SCL(1);
        if (MI2C_R_SDA() != 0U)
        {
            Byte |= (uint8_t)(0x80U >> i);
        }
        MI2C_W_SCL(0);
    }
    return Byte;
}

void MI2C_SendAck(uint8_t AckBit)
{
    MI2C_W_SDA(AckBit);
    MI2C_W_SCL(1);
    MI2C_W_SCL(0);
}

uint8_t MI2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MI2C_W_SDA(1);
    MI2C_W_SCL(1);
    AckBit = MI2C_R_SDA();
    MI2C_W_SCL(0);
    return AckBit;
}
