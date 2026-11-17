/**
  * @file    mpu6050.c
  * @brief   MPU6050 六轴姿态：寄存器读写 + 互补滤波解算 yaw/pitch/roll
  *
  * 队友原文件：C:\Users\zh\Desktop\11-1MPU - 副本\Hardware\MPU6050.c（标准库 + 软件 I2C + TIM3 节拍）
  *
  * 移植改动汇总（除下面这些，解算算法一行没动）：
  *   1) 通信：标准库软件 I2C（PB10/PB11）
  *            → 硬件 I2C1：HAL_I2C_Mem_Write / HAL_I2C_Mem_Read(&hi2c1, ...)
  *            （PB10/PB11 在本工程是电机方向脚 AIN1/AIN2，物理上不能占）
  *   2) 节拍：TIM3 + TIM3_IRQHandler + s_tick
  *            → HAL_GetTick() 判断，MPU6050_TickFlag() 每 10ms 返回 1（接口不变）
  *   3) 延时：Delay_ms(4)
  *            → HAL_Delay(4)
  *   4) 时间戳：*(0xE0001004)/72
  *            → DWT->CYCCNT / (SystemCoreClock/1000000)   （主频换 72/64/48 都对）
  *   5) 去掉了重复定义的 TIM3_IRQHandler（本工程中断入口在 stm32f1xx_it.c，
  *      里面自动调 HAL_TIM_PeriodElapsedCallback，再自己写一份会链接冲突）
  */
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "mi2c.h"
#include <math.h>

#if (MPU6050_USE_HW_I2C != 0)
extern I2C_HandleTypeDef hi2c1;      /* CubeMX 生成的 I2C1 句柄（PB8=SCL, PB9=SDA） */
#endif

/* ===================== 内部状态（和队友一致） ===================== */
static float yaw   = 0.0f;
static float pitch = 0.0f;
static float roll  = 0.0f;

static float gx = 0.0f, gy = 0.0f, gz = 0.0f;
static float ax = 0.0f, ay = 0.0f, az = 0.0f;
static float temperature = 0.0f;

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float yaw_rate      = 0.0f;

/* 单位重力向量（车体坐标系下指向"世界竖直方向"），用加速度计在线估计。
 * MPU 装平、装竖、装斜都无所谓——车原地转的转轴一定是世界竖直方向。 */
static float   grav_u[3] = {0.0f, 0.0f, 1.0f};
static uint8_t grav_ready = 0U;

static uint8_t s_online = 0U;

/* ===================== 不用串口也能看的调试变量 =====================
 * 串口一时通不了的时候，在 Keil 的 Debug 模式里把下面这几个名字加进
 * Watch 窗口（View → Watch Windows → Watch 1），全速跑起来就能看到实时值：
 *   g_dbg_online   1 = I2C 通了（读到了 WHO_AM_I=0x68），0 = 陀螺仪没接上
 *   g_dbg_yaw      当前航向角（度），用手把车转 90° 它就变 ~90
 *   g_dbg_tilt     模块安装倾斜角（度），看到非 0 就说明投影补偿在起作用
 *   g_dbg_yaw_rate 绕竖直轴的角速度（度/秒），静止应接近 0
 * 这几个变量只写不读，不会被编译器优化掉。 */
volatile uint8_t g_dbg_online   = 0U;
volatile float   g_dbg_yaw      = 0.0f;
volatile float   g_dbg_tilt     = 0.0f;
volatile float   g_dbg_yaw_rate = 0.0f;

/* ===================== 寄存器读写 =====================
 * 地址用 s_addr8（运行时变量）而不是写死的 0x68：
 * GY-521 这类模块的 AD0 脚悬空时地址可能是 0x69，Init 里会自动探测两个地址。
 * 所有读失败都会把 s_i2c_err 加一，方便在 Watch 窗口判断"到底通没通"。 */
static uint8_t  s_addr8   = MPU6050_ADDR_8BIT;   /* 当前使用的 8 位地址（含读写位 0） */
static uint8_t  s_who     = 0U;                  /* 最近一次读到的 WHO_AM_I 原始值 */
static uint32_t s_i2c_err = 0U;                  /* 累计 I2C 失败次数 */

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
#if (MPU6050_USE_HW_I2C != 0)
    /* 标准库：Start + 地址 + 应答 + 寄存器 + 应答 + 数据 + 应答 + Stop
       HAL 版：一个 HAL_I2C_Mem_Write 全包了 */
    if (HAL_I2C_Mem_Write(&hi2c1, s_addr8, RegAddress,
                          I2C_MEMADD_SIZE_8BIT, &Data, 1U, MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
        s_i2c_err++;
    }
#else
    MI2C_Start();
    MI2C_SendByte(s_addr8);
    MI2C_ReceiveAck();
    MI2C_SendByte(RegAddress);
    MI2C_ReceiveAck();
    MI2C_SendByte(Data);
    MI2C_ReceiveAck();
    MI2C_Stop();
#endif
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
#if (MPU6050_USE_HW_I2C != 0)
    uint8_t Data = 0U;
    if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, RegAddress,
                         I2C_MEMADD_SIZE_8BIT, &Data, 1U, MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
        s_i2c_err++;
        return 0xFFU;                   /* 0xFF = 读失败（和真实数据 0x00 区分开） */
    }
    return Data;
#else
    uint8_t Data;
    MI2C_Start();
    MI2C_SendByte(s_addr8);
    MI2C_ReceiveAck();
    MI2C_SendByte(RegAddress);
    MI2C_ReceiveAck();

    MI2C_Start();
    MI2C_SendByte(s_addr8 | 0x01);
    MI2C_ReceiveAck();
    Data = MI2C_ReceiveByte();
    MI2C_SendAck(1);
    MI2C_Stop();

    return Data;
#endif
}

/* 连续读 Len 个字节：从 RegAddress 开始，芯片寄存器指针自动 +1
 * 返回 1 = 成功，0 = 失败（失败时 Buf 全填 0，绝不留栈上的垃圾数据） */
static uint8_t MPU6050_ReadBurst(uint8_t RegAddress, uint8_t *Buf, uint8_t Len)
{
    uint8_t i;

#if (MPU6050_USE_HW_I2C != 0)
    for (i = 0U; i < Len; i++)
    {
        Buf[i] = 0U;                    /* 先清零：读失败时不会把垃圾当数据算 */
    }
    if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, RegAddress,
                         I2C_MEMADD_SIZE_8BIT, Buf, (uint16_t)Len, MPU6050_I2C_TIMEOUT) != HAL_OK)
    {
        s_i2c_err++;
        for (i = 0U; i < Len; i++)
        {
            Buf[i] = 0U;
        }
        return 0U;
    }
    return 1U;
#else
    MI2C_Start();
    MI2C_SendByte(s_addr8);
    MI2C_ReceiveAck();
    MI2C_SendByte(RegAddress);
    MI2C_ReceiveAck();
    MI2C_Start();
    MI2C_SendByte(s_addr8 | 0x01);
    MI2C_ReceiveAck();
    for (i = 0U; i < Len; i++)
    {
        Buf[i] = MI2C_ReceiveByte();
        MI2C_SendAck((i == (uint8_t)(Len - 1U)) ? 1U : 0U);   /* 最后一个回 NACK */
    }
    MI2C_Stop();
    return 1U;
#endif
}

/* ---------------- I2C 探测（排障用） ---------------- */
uint8_t MPU6050_ProbeAddr(uint8_t addr7)
{
    uint8_t t;
    uint8_t found = 0U;

    for (t = 0U; t < 3U; t++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2U, 20U) == HAL_OK)
        {
            found = 1U;
            break;
        }
    }
    return found;
}

/* 重新探测：AD0 悬空的模块可能是 0x69，两个都试一遍 */
uint8_t MPU6050_Ping(void)
{
    if (MPU6050_ProbeAddr(MPU6050_ADDR_7BIT) != 0U)
    {
        s_addr8 = (uint8_t)(MPU6050_ADDR_7BIT << 1);
        s_online = 1U;
    }
    else if (MPU6050_ProbeAddr(MPU6050_ADDR_7BIT_ALT) != 0U)
    {
        s_addr8 = (uint8_t)(MPU6050_ADDR_7BIT_ALT << 1);   /* AD0 接高：0x69 */
        s_online = 1U;
    }
    else
    {
        s_online = 0U;
    }

    if (s_online != 0U)
    {
        s_who = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    }
    g_dbg_online = s_online;
    return s_online;
}

uint8_t MPU6050_WhoAmIRaw(void)  { return s_who; }
uint8_t MPU6050_GetAddr7(void)   { return (uint8_t)(s_addr8 >> 1); }
uint32_t MPU6050_ErrCount(void)  { return s_i2c_err; }

/* 总线上扫一遍 0x08~0x77，把有应答的地址写进数组。
 * 用途：下载后串口只看到 "NOT FOUND" 时，用它区分
 *   - 一个地址都没有 → 根本没通（没供电 / 没上拉 / SCL/SDA 接反或接错脚 / 线断）
 *   - 只有 0x68 或 0x69 → 器件在，只是地址或配置问题
 *   - 冒出别的地址 → 挂错线了（比如接到别的 I2C 器件）
 * 每个地址只试 1 次（20ms），扫完整条总线最坏约 2.2s，只在排障时调。 */
uint8_t MPU6050_ScanBus(uint8_t *Found, uint8_t MaxFound)
{
    uint8_t addr7;
    uint8_t cnt = 0U;

    for (addr7 = 0x08U; addr7 <= 0x77U; addr7++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 1U, 20U) == HAL_OK)
        {
            if (cnt < MaxFound)
            {
                Found[cnt] = addr7;
            }
            cnt++;
        }
    }
    return cnt;
}

/* ===================== DWT 时间戳 ===================== */
void MPU6050_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能系统跟踪 */
    DWT->CYCCNT = 0U;                                 /* 清周期计数器 */
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动计数器 */
}

uint32_t MPU6050_GetUs(void)
{
    uint32_t per_us = (uint32_t)(SystemCoreClock / 1000000U);
    if (per_us == 0U)
    {
        per_us = 72U;
    }
    return DWT->CYCCNT / per_us;
}

/* ===================== 10ms 节拍（原来是 TIM3 中断） ===================== */
void MPU6050_TimeBaseInit(void)
{
    /* 本工程 TIM3 已由 CubeMX 配成 10ms 并在 main.c 的
       HAL_TIM_PeriodElapsedCallback 里处理，这里什么都不用做。
       需要 10ms 标志就调 MPU6050_TickFlag()。 */
}

uint8_t MPU6050_TickFlag(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) >= (uint32_t)MPU6050_TICK_MS)
    {
        last_tick = now;
        return 1U;
    }
    return 0U;
}

/* ===================== 初始化 ===================== */
void MPU6050_Init(void)
{
#if (MPU6050_USE_HW_I2C == 0)
    MI2C_Init();                        /* 软件 I2C 才需要自己配 GPIO */
#endif
    /* 先探测器件在不在（0x68 / 0x69 都试），不在也照样写配置寄存器——
       写不进去只是计到 s_i2c_err 里，不会卡住。 */
    (void)MPU6050_Ping();

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1,  0x01);   /* 退出睡眠，用 X 轴陀螺 PLL */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2,  0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV,  0x09);   /* 1kHz/(1+9) = 100Hz */
    MPU6050_WriteReg(MPU6050_CONFIG,      0x03);   /* DLPF 44Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);   /* ±500 dps → 65.5 LSB/(°/s) */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);   /* ±2g → 16384 LSB/g */

    MPU6050_DwtInit();

    if (s_online != 0U)
    {
        s_who = MPU6050_ReadReg(MPU6050_WHO_AM_I);  /* MPU6050 = 0x68；0x70=MPU6500、0x71/0x73=MPU925x */
    }
    g_dbg_online = s_online;            /* Keil Watch 窗口直接看这个判断 I2C 通不通 */
}

uint8_t MPU6050_IsOnline(void)
{
    return s_online;
}

/* ===================== 采数据 ===================== */
void MPU6050_GetData(void)
{
    uint8_t b[14];

    if (MPU6050_ReadBurst(MPU6050_ACCEL_XOUT_H, b, 14U) == 0U)
    {
        /* 读失败：全部清 0，绝不能让栈上的垃圾当成加速度/角速度去积分 */
        ax = ay = az = 0.0f;
        gx = gy = gz = 0.0f;
        temperature = 0.0f;
        return;
    }

    ax          = (float)(int16_t)(((uint16_t)b[0] << 8) | b[1]) / 16384.0f;
    ay          = (float)(int16_t)(((uint16_t)b[2] << 8) | b[3]) / 16384.0f;
    az          = (float)(int16_t)(((uint16_t)b[4] << 8) | b[5]) / 16384.0f;
    temperature = (float)(int16_t)(((uint16_t)b[6] << 8) | b[7]) / 340.0f + 36.53f;
    gx          = (float)(int16_t)(((uint16_t)b[8] << 8) | b[9])  / 65.5f;
    gy          = (float)(int16_t)(((uint16_t)b[10] << 8) | b[11]) / 65.5f;
    gz          = (float)(int16_t)(((uint16_t)b[12] << 8) | b[13]) / 65.5f;
}

/* 静止零偏标定：500 次 × 4ms ≈ 2s，期间车必须完全不动 */
void MPU6050_Calibrate(uint16_t number)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint16_t i;

    for (i = 0U; i < number; i++)
    {
        MPU6050_GetData();
        sx += gx;
        sy += gy;
        sz += gz;
        HAL_Delay(4);
    }
    gyro_bias[0] = sx / (float)number;;
    gyro_bias[1] = sy / (float)number;;
    gyro_bias[2] = sz / (float)number;;
}

/* ===================== 进程函数（解算） ===================== */
void MPU6050_Proc(void)
{
    static uint32_t last_us    = 0U;
    static uint8_t  static_cnt = 0U;

    uint32_t now_us;
    float dt;
    float wx, wy, wz, acc;
    float roll_acc, pitch_acc;
    float gx_u, gy_u, gz_u, gm;

    MPU6050_GetData();

    now_us = MPU6050_GetUs();
    dt = (float)(uint32_t)(now_us - last_us) * 1.0e-6f;   /* 无符号相减，计数器回绕也正确 */
    last_us = now_us;
    if ((dt <= 0.0f) || (dt > 0.2f))
    {
        dt = 0.01f;                                       /* 异常值兜底 */
    }

    /* 去掉初始零偏 */
    wx = gx - gyro_bias[0];
    wy = gy - gyro_bias[1];
    wz = gz - gyro_bias[2];

    /* 静止检测：角速度小 且 合加速度 ≈ 1g */
    acc = sqrtf(ax * ax + ay * ay + az * az);
    if ((fabsf(wx) < 1.0f) && (fabsf(wy) < 1.0f) && (fabsf(wz) < 1.0f) && (fabsf(acc - 1.0f) < 0.15f))
    {
        if (static_cnt < 20U)
        {
            static_cnt++;
        }
    }
    else
    {
        static_cnt = 0U;
    }

    /* 静止时缓慢修正零偏 */
    if (static_cnt >= 20U)
    {
        gyro_bias[0] += (gx - gyro_bias[0]) * 0.002f;
        gyro_bias[1] += (gy - gyro_bias[1]) * 0.002f;
        gyro_bias[2] += (gz - gyro_bias[2]) * 0.002f;
        wx = 0.0f;
        wy = 0.0f;
        wz = 0.0f;
    }

    /* 抑制静止时来回抖动累积成缓慢旋转 */
    if (fabsf(wx) < 0.6f) { wx = 0.0f; }
    if (fabsf(wy) < 0.6f) { wy = 0.0f; }
    if (fabsf(wz) < 0.6f) { wz = 0.0f; }

    /* 加速度计给绝对倾角 */
    roll_acc  = atan2f(ay, az) * 57.29578f;
    pitch_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;

    /* 互补滤波：陀螺积分 + 慢速修正 */
    roll  = 0.98f * (roll  + wx * dt) + 0.02f * roll_acc;
    pitch = 0.98f * (pitch + wy * dt) + 0.02f * pitch_acc;

    /* ---------------- 在线估计"世界竖直方向"在车体坐标里的指向 ----------------
     * 静止/匀速时加速度计测到的 1g 向量就是世界竖直方向（车体坐标下表示）。
     * 用低通把车体振动滤掉，再归一化成单位向量 grav_u[]。
     * |acc| 不在 0.7~1.3g 之间（急加速/撞车/掉线）就不更新，用上一次的值。
     */
    if ((acc > 0.70f) && (acc < 1.30f))
    {
        gx_u = ax / acc;
        gy_u = ay / acc;
        gz_u = az / acc;

        if (grav_ready == 0U)
        {
            grav_u[0] = gx_u;                   /* 第一次直接采信 */
            grav_u[1] = gy_u;
            grav_u[2] = gz_u;
            grav_ready = 1U;
        }
        else
        {
            grav_u[0] += (gx_u - grav_u[0]) * MPU6050_GRAV_LPF;
            grav_u[1] += (gy_u - grav_u[1]) * MPU6050_GRAV_LPF;
            grav_u[2] += (gz_u - grav_u[2]) * MPU6050_GRAV_LPF;

            gm = sqrtf(grav_u[0] * grav_u[0] + grav_u[1] * grav_u[1] + grav_u[2] * grav_u[2]);
            if (gm > 0.001f)                    /* 低通会掉模长，重新归一化 */
            {
                grav_u[0] /= gm;
                grav_u[1] /= gm;
                grav_u[2] /= gm;
            }
        }
    }

    /* ---------------- yaw：把角速度投影到竖直轴上积分 ----------------
     * 原地转向绕的就是竖直轴，所以只要把三轴角速度投影到 grav_u 上：
     *     yaw_rate = wx*gx + wy*gy + wz*gz
     * MPU 装斜了也不影响：斜着装时 gx/gy 分量会自己把 wx/wy 里那部分
     * "跟着挠"的转动抵掉，只有真正绕竖直轴的转动被积进去。
     */
#if (MPU6050_TILT_FREE_YAW != 0)
    if (grav_ready != 0U)
    {
        yaw_rate = wx * grav_u[0] + wy * grav_u[1] + wz * grav_u[2];
    }
    else
    {
        yaw_rate = wz;                      /* 还没采到重力，先按水平安装算 */
    }
#else
    {
        float sr = sinf(roll  / 57.29578f);
        float cr = cosf(roll  / 57.29578f);
        float cp = cosf(pitch / 57.29578f);

        if (cp < 0.17f)
        {
            cp = 0.17f;                     /* ±80° 之外不积分 */
        }
        yaw_rate = (wy * sr + wz * cr) / cp;
    }
#endif

    if (fabsf(yaw_rate) < 0.3f)
    {
        yaw_rate = 0.0f;                    /* 投影后的死区，抑制漂 */
    }
    if (static_cnt >= 20U)
    {
        yaw_rate = 0.0f;                    /* 完全静止就彻底冻住，防止慢飘 */
    }
    yaw += yaw_rate * dt;
    if (yaw > 180.0f)
    {
        yaw -= 360.0f;
    }
    if (yaw < -180.0f)
    {
        yaw += 360.0f;
    }

    /* ---- 调试镜像：串口不通时用 Keil 的 Watch 窗口看这几个 ---- */
    g_dbg_yaw      = yaw;
    g_dbg_yaw_rate = yaw_rate;
    {
        float c = grav_u[2];                /* 世界竖直方向在模块 Z 轴上的投影 */

        if (c > 1.0f)  { c = 1.0f; }
        if (c < -1.0f) { c = -1.0f; }
        g_dbg_tilt = (grav_ready != 0U) ? (acosf(c) * 57.29578f) : 0.0f;
    }
}

/* ===================== 读取接口 ===================== */
float MPU6050_AXData(void)      { return ax; }
float MPU6050_AYData(void)      { return ay; }
float MPU6050_AZData(void)      { return az; }
float MPU6050_GXData(void)      { return gx; }
float MPU6050_GYData(void)      { return gy; }
float MPU6050_GZData(void)      { return gz; }
float MPU6050_TempData(void)    { return temperature; }

float MPU6050_YawData(void)     { return yaw; }
float MPU6050_PitchData(void)   { return pitch; }
float MPU6050_RollData(void)    { return roll; }
float MPU6050_YawRateData(void) { return yaw_rate; }

/* 安装倾斜角（度）= 模块自身 Z 轴和世界竖直方向的夹角。
 * 只用于自检打印：0°≈平放，90°≈竖着贴。yaw 已做投影补偿，这个角多大都不影响精度。 */
float MPU6050_MountTiltDeg(void)
{
    float c;

    if (grav_ready == 0U)
    {
        return 0.0f;
    }

    c = grav_u[2];                          /* 世界竖直方向在模块 Z 轴上的投影 */
    if (c > 1.0f)
    {
        c = 1.0f;
    }
    if (c < -1.0f)
    {
        c = -1.0f;
    }
    return acosf(c) * 57.29578f;
}

float MPU6050_BiasXData(void)   { return gyro_bias[0]; }
float MPU6050_BiasYData(void)   { return gyro_bias[1]; }
float MPU6050_BiasZData(void)   { return gyro_bias[2]; }

void  MPU6050_ResetYaw(void)    { yaw = 0.0f; yaw_rate = 0.0f; }
void  MPU6050_SetYaw(float deg) { yaw = deg; }
