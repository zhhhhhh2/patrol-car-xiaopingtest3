# 智能循迹小车（STM32F103）

基于 STM32F103C8T6 + STM32 HAL 库的循迹小车固件。赛道为矩形闭环，小车需要完成直行保直、弧线循迹、定点转向和多圈连跑，由状态机统一调度。

## 硬件

| 模块 | 型号 / 接口 | 说明 |
| --- | --- | --- |
| MCU | STM32F103C8T6 | Cortex-M3，72 MHz |
| 电机驱动 | TB6612 | TIM1 输出 PWM，支持正反转与 STBY 使能 |
| 循迹 | 7 路灰度传感器 | 数字量输出，用于计算线偏差 |
| 姿态 | MPU6050 | 软件 I2C，提供 yaw 角 |
| 人机交互 | 按键 / LED / 蜂鸣器 | 选题、急停、状态提示 |

## 功能

- **状态机调度**：`IDLE → RUN → STOP`，每 10 ms 跑一次 `Task_Update()`
- **直行保直**：陀螺仪 yaw 误差 PD 修正，左右轮差速
- **原地转向**：`Task_Heading_Turn()` 转到目标角后锁存，非阻塞、带超时
- **灰度循迹**：7 路灰度 → 偏差 → 分档差速
- **顶点检测**：双沿触发 + 消抖（30 ms），4 个顶点对应 5 个阶段
- **按键选题**：上电后按 N 下跑第 N 题，跑动中再按一次急停

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `Core/` | CubeMX 生成的初始化与中断（`main.c` 只保留 `USER CODE` 段） |
| `car_driver/` | 驱动层：`task.c` 状态机、`sensor.c` 灰度、`moter.c` 电机 PWM、`mpu6050.c` 陀螺仪、`key.c` / `led.c` |
| `MDK-ARM/` | Keil 工程（`xiaopingtest.uvprojx`）、启动文件、队友的 `turn.c` / `pid.c` / `uart.c` |
| `Drivers/` | STM32F1xx HAL 驱动与 CMSIS |
| `xiaopingtest.ioc` | STM32CubeMX 工程配置 |

## 编译与烧录

1. 用 Keil MDK-ARM 5 打开 `MDK-ARM/xiaopingtest.uvprojx`
2. Rebuild（需要安装 STM32F1xx DFP 器件包）
3. 通过 ST-Link / DAP-Link 下载到板子

引脚与时钟配置在 `xiaopingtest.ioc` 中，用 CubeMX 打开即可修改。

## 文档

- [`思路总览_整合后怎么读.md`](思路总览_整合后怎么读.md)：状态机、赛道顶点、参数说明与排障表
- [`整合说明_标准库转HAL.md`](整合说明_标准库转HAL.md)：从标准库迁移到 HAL 的过程记录

## 备注

- 电机 PWM 的唯一写入者是 `task.c` 的 `Task_Motor_Set()`，中断里不再写电机，避免两处输出互相对冲
- 调试阶段可把 `main.c` 顶部的 `USE_TASK_TURN_TEST` 置 1，上电自动转 90°，用来单独验证转向参数
