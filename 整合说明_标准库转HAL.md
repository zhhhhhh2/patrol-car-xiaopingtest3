# 队友代码整合说明（标准库 → HAL）

> 目标工程：`C:\Users\zh\Desktop\xiaopingtest`（CubeMX + Keil MDK-ARM）
> 来源工程：`C:\Users\zh\Desktop\灰度传感器寻迹6 - 副本`（标准库）

---

## 一、文件清单（都放到 `xiaopingtest\car_driver\`）

| 文件 | 作用 | 对应队友原文件 |
|---|---|---|
| `sensor.c` / `sensor.h` | 7 路灰度循迹：读原始值、加权偏差、状态判定、查表差速 | `hardware\Sensor.c/.h` |
| `key.c` / `key.h` | 启动按键（消抖、非阻塞扫描） | `hardware\key.c/.h` |
| `led.c` / `led.h` | 板载 LED + 蜂鸣器（声光提示） | `hardware\LED.c/.h` |
| `task.c` / `task.h` | **状态机**（从 `user\main.c` 整段剥离） | `user\main.c` 的 while(1) |
| `mpu6050.c` / `mpu6050.h` / `mpu6050_reg.h` | 陀螺仪 MPU6050：寄存器读写 + 互补滤波解算 yaw/pitch/roll | `Hardware\MPU6050.c/.h`、`MPU6050_Reg.h` |
| `mi2c.c` / `mi2c.h` | 软件 I2C 位操作（**备胎**，默认走硬件 I2C1，不用它） | `Hardware\MI2C.c/.h` |
| `Core\Src\main.c` | 瘦身后的 main，只留 CubeMX 结构 + 初始化 + `Task_Update()` | — |

`car_driver` 已经在 Keil 的 Include Path 里，所以头文件直接 `#include "sensor.h"` 即可。
六个新 `.c`（sensor/key/led/task/mpu6050/mi2c）已经写进 `MDK-ARM\xiaopingtest.uvprojx` 的 `car_driver` 组（打开 Keil 就能看到，直接编译）。

---

## 二、标准库 → HAL 对照（代码里也逐行写了注释）

| 队友（标准库） | 我们（HAL） |
|---|---|
| `RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA\|GPIOB, ENABLE)` | 不需要，`MX_GPIO_Init()` 已使能 |
| `GPIO_Mode_IPU` | `Mode = GPIO_MODE_INPUT; Pull = GPIO_PULLUP` |
| `GPIO_Mode_Out_PP` | `Mode = GPIO_MODE_OUTPUT_PP` |
| `GPIO_Init(GPIOA, &s)` | `HAL_GPIO_Init(GPIOA, &s)` |
| `GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)` | `HAL_GPIO_ReadPin(RED1_GPIO_Port, RED1_Pin)` |
| `GPIO_ResetBits(...)` / `GPIO_SetBits(...)` | `HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET/SET)` |
| `Delay_ms(20)` | `HAL_Delay(20)` |
| `#include "stm32f10x.h"` | `#include "stm32f1xx_hal.h"` + `#include "main.h"` |

### 引脚对照（队友 → 我们，以 `Core\Inc\main.h` 的 CubeMX 宏为准）

| 功能 | 队友引脚 | 我们的宏 |
|---|---|---|
| 灰度 OUT7（最左） | PB0 | `RED7_Pin` |
| OUT6 ~ OUT1 | PA7 ~ PA2 | `RED6_Pin` ~ `RED1_Pin` |
| （第 8 路预留） | — | `RED8_Pin`（PB1，暂未用） |
| 启动键 | PB15 / PB1 | `KEY_Pin`（**当前 PC15**） |
| LED | PA1、PA2 | `LED_Pin`（PC13，低电平亮） |
| 蜂鸣器 | 无 | `Buzzer_Pin`（PB5，高电平响） |

**⚠ 需要现场确认的 3 处**

1. **按键到底接 PC15 还是 PB15？** CubeMX 配的是 **PC15**，队友代码/引脚表写的是 PB15。确认后：如果是 PB15，在 CubeMX 里把 KEY 挪到 PB15 重新生成（或手改 `main.h` 的 `KEY_Pin`/`KEY_GPIO_Port`），驱动代码一行都不用动。
2. **灰度模块是 7 路还是 8 路？** 队友代码只用了 7 路，我们预留了 RED8。
3. **黑白电平极性**：`sensor.h` 里 `SENSOR_BLACK_IS_HIGH`，如果实测"压黑线读到 0"，把它改成 0 就行，逻辑不用动。

---

## 三、修掉的 5 个 bug（队友原 main.c）

| # | 原代码问题 | 改法 |
|---|---|---|
| 1 | `#elif (CURRENT_TRIGGER_MODE == TRIGGER_ON_LINE_TO_LOST)` —— 两个宏根本没定义，C 里未定义标识符在 `#if` 中当 0，两个 0 相等，**条件恒成立**，等于触发模式永远被强制成某一种 | 用 `TASK_TRIGGER_MODE` 明确选边沿，默认 `TASK_TRIG_ON_BOTH`（两条边沿都算一个顶点） |
| 2 | 每个答题分支末尾都有 `point_count = 0;`，**每轮都清零，计数永远到不了 2**，第 2 题以后的分支永远进不去 | 只在"换圈/换题"时清零 |
| 3 | 第 4 题 `for (i = 0; i < 3; i++) { switch (...) }` —— switch 里没有 break 出去做循环，等于只跑一遍 | 用 `s_lap_count` 计圈，跑满 `TASK_PROBLEM4_LAPS`（4 圈）才停 |
| 4 | 大括号 / switch 错位，末尾多一层 `switch` 和多余 `}` | 重写为规整的 `switch (s_key_count) → switch (s_current_task)` 两层结构 |
| 5 | `Check_PointEvent()` 在 LOST_TO_LINE 模式下"检测到事件"也 `return 0`，和"没事件"无法区分 | 统一语义：`return 1` 恒表示"这一次识别到一个顶点" |

另外顺手做的改进：

- **声光提示改成非阻塞**：`Task_Beep(ms)` 只记一个时间戳，`Task_Update()` 每拍轮询关闭。原版 `Delay_ms(100)` 会把循迹卡停 100ms，轮子保持上一次的 PWM 直接冲出弧线。
- **状态机自节流**：`Task_Update()` 内部用 `HAL_GetTick()` 卡 10ms，主循环里 `HAL_Delay` 全部删掉，按键响应最快。
- **过点判定有消抖**：连续 3 拍（30ms）状态一致才认，防止弧线边缘抖动误判成顶点。
- **第 1 题有 15s 限时保护**：超时自动停车，避免一直顶墙。

---

## 四、状态机怎么工作（`task.c`）

```
上电 → IDLE（LED 灭）
        │  按键 1 下 = 第 1 题，2 下 = 第 2 题，3 下 = 第 3 题，4 下 = 第 4 题
        │  （第 5 下循环回第 1 题）
        ▼
      RUN ──每 10ms：采 7 路灰度 → 过点判定 → 执行当前阶段
        │            └─ 识别到顶点：point_count++，蜂鸣器短鸣一声
        │  运行中再按一次键 = 紧急停车
        ▼
      STOP（LED 灭）──再按键 → 回到 IDLE，可以接着选下一题
```

### 顶点编号 ↔ 阶段

默认双沿触发，所以**一个物理顶点只触发一次**，正好对上队友注释里的 A/B/C/D：

| point_count | 阶段枚举 | 含义 |
|---|---|---|
| 0 | `TASK_RUN_PROBLEM_1` | A 点起步（A→B 盲走 / A 点转向） |
| 1 | `TASK_RUN_PROBLEM_2` | B 点：进第 1 段弧 |
| 2 | `TASK_RUN_PROBLEM_3` | C 点：出第 1 段弧 |
| 3 | `TASK_RUN_PROBLEM_4` | D 点：进第 2 段弧 |
| 4 | `TASK_RUN_PROBLEM_5` | 回到 A 点 |

### 四道题的动作

| 题 | 流程 |
|---|---|
| 第 1 题 | A→B 盲走直行 → 到 B 停车（限时 15s） |
| 第 2 题 | A→B 直行 → B 进弧 `Sensor_Find` 循迹 → C 出弧直行 → D 进弧循迹 → 回 A 停车 |
| 第 3 题 | A 转向 → 直走到 C → C 进弧循迹 → B 转向 → 直走到 D → D 进弧循迹 → 回 A 停车 |
| 第 4 题 | 同第 3 题路径，连跑 4 圈后停车 |

要改触发方式，只改 `task.h` 一行：

```c
#define TASK_TRIGGER_MODE   TASK_TRIG_ON_BOTH        // 默认：每个顶点一次
// #define TASK_TRIGGER_MODE  TASK_TRIG_ON_LINE_TO_LOST  // 只在"刚出黑线"触发
// #define TASK_TRIGGER_MODE  TASK_TRIG_ON_LOST_TO_LINE  // 只在"刚进黑线"触发
```

要改速度 / 消抖 / 圈数，都在 `task.h` 顶部的宏里：

```c
#define TASK_TICK_MS            10      // 状态机节拍
#define TASK_DEBOUNCE_THRES     3       // 3×10ms = 30ms 过点消抖
#define TASK_BLIND_SPEED        300     // 盲走/直行 PWM
#define TASK_ARC_BASE_SPEED     300     // 弧线循迹基础 PWM
#define TASK_PROBLEM4_LAPS      4       // 第 4 题跑几圈
```

---

## 五、新的 `main.c` 长什么样

只保留 CubeMX 骨架，改动全在 USER CODE 区（重新用 CubeMX 生成也不会丢）：

```c
/* USER CODE BEGIN Includes */
#include "team.h"
#include "sensor.h"
#include "key.h"
#include "led.h"
#include "task.h"
/* USER CODE END Includes */
...
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   /* STBY */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim3);              /* 10ms */

	Sensor_Init();
	Key_Init();
	LED_Init();
	Buzzer_Init();
	Task_Init();
  /* USER CODE END 2 */

  while (1)
  {
	Task_Update();        /* 状态机，内部 10ms 节流 */
    /* USER CODE END WHILE */
  }
```

原来的 `Debug_Send4Int(...) + HAL_Delay(100)` 已注释掉（它每 100ms 阻塞一次，会把循迹节奏打乱）。要调试时把 while 里那段带 500ms 闸门的注释打开即可，发的是编码器计数 + 当前左右 PWM。

---

## 六、还要队友补的一块（代码里留了 `TODO(陀螺仪)`）

1. **陀螺仪航向环**：`turn.c` 里 `BSP_IMU_GetYaw()` 原来是个 `return 0` 的空壳 —— **已解决**，现在返回 `MPU6050_YawData()`（见第八节）。`task.c` 里"直行/转向"分支目前仍是**等速直行占位**，陀螺仪通了以后把这几处换成航向 PI 输出的差速即可，接口位置都标好了。
2. **速度闭环**：`pid.c` 的 `SpeedLoop_Update()` 在 TIM3 中断里仍是注释状态。**现在状态机是开环直接给 PWM，不要同时打开速度环**，否则两处都在写 `Motor_Output_L/R`，会互相打架。如果要用闭环，把 `task.c` 里的 `Task_Motor_Set()` 改成"写目标速度"，其余不用动。

---

## 七、编译注意

- PWM：TIM1 `PSC=3, ARR=999` → 18kHz，比较值有效范围 `0 ~ 1000`（写 1000 即满占空比），`Sensor_Find()` 输出已经限幅在 0~1000。
- 六个新文件已在 `xiaopingtest.uvprojx` 的 `car_driver` 组里；如果 Keil 里看不到，手动 Add Existing Files 加到该组即可。
- 头文件包含顺序：`sensor.c/key.c/led.c/task.c/mpu6050.c/mi2c.c` 都 `#include "main.h"`，所以必须保证 `main.h` 先被正确处理（CubeMX 工程本来就是）。
- 编译前把旧的 `main.c` 备份（整合时已自动备份成 `main.c.bak`）。
- `mpu6050.c` 里用到的 `CoreDebug` / `DWT` 来自 `core_cm3.h`，CubeMX 工程自动包含，不用额外加文件。

---

## 八、陀螺仪（队友 C 的 MPU6050）怎么接进来的

### 8.1 关键冲突：不能沿用队友原来的 PB10/PB11

队友 `MI2C.c` 把 SCL 放在 **PB10**、SDA 放在 **PB11**；
我们这块板子上 **PB10 = AIN1、PB11 = AIN2**（TB6612 的电机方向脚），
所以陀螺仪**必须换两个脚**，软件 I2C 原样照抄是跑不起来的。

本工程 CubeMX 里 I2C1 已经配好了（重映射到 PB8/PB9，100 kHz）：

- `Core\Src\stm32f1xx_hal_msp.c`：`PB8 ------> I2C1_SCL`、`PB9 ------> I2C1_SDA`，`__HAL_AFIO_REMAP_I2C1_ENABLE()`
- 所以 **陀螺仪接 PB8(SCL) / PB9(SDA)** 就行，CubeMX 配置一个字都不用改。

⚠ 上电前确认：GY-521 模块自带 4.7k 上拉；如果用的是裸芯片，SCL/SDA 各要一个 4.7k~10k 上拉到 3.3V。

### 8.2 默认走硬件 I2C，软件 I2C 当备胎

`mpu6050.h` 顶部：

```c
#define MPU6050_USE_HW_I2C      1   /* 1 = 硬件 I2C1（推荐）；0 = 用 mi2c.c 的软件位操作 */
```

真要改用软件 I2C（比如陀螺仪只能接别的脚），在 `mi2c.h` 顶部改：

```c
#define MI2C_SCL_GPIO_PORT   GPIOB
#define MI2C_SCL_GPIO_PIN    GPIO_PIN_8
#define MI2C_SDA_GPIO_PORT   GPIOB
#define MI2C_SDA_GPIO_PIN    GPIO_PIN_9
```

现已被占用的脚（别选）：PA2~PA8、PA11、PB0/PB1、PB5、PB6/PB7（TIM4 编码器）、PB8/PB9（I2C1）、PB10~PB14、PC13、PC15。

### 8.3 标准库 → HAL 改动清单

| 队友（标准库） | 我们（HAL） |
|---|---|
| `MI2C_*` 位操作 + `GPIO_WriteBit` / `GPIO_ReadInputDataBit` | 默认 `HAL_I2C_Mem_Read/Write(&hi2c1, ...)`；软件模式用 `HAL_GPIO_WritePin/ReadPin` |
| `GPIO_Mode_Out_OD` | `GPIO_MODE_OUTPUT_OD` |
| `TIM3` 10ms + 自己写的 `TIM3_IRQHandler` | **删掉**（本工程中断入口在 `stm32f1xx_it.c`，重复定义会链接冲突）；改用 `HAL_GetTick()`，`MPU6050_TickFlag()` 每 10ms 返回 1 |
| `Delay_ms(4)` | `HAL_Delay(4)` |
| `MPU6050_GetUs()` = `*(0xE0001004)/72` | `DWT->CYCCNT / (SystemCoreClock/1000000)`，主频改了对得上 |
| 主循环里自己判断节拍调 `MPU6050_Proc()` | `Task_Update()` 每 10ms 自动调一次（`task.h` 的 `TASK_IMU_ENABLE` 可关） |

`MPU6050_Proc()` 里的解算算法（静止检测、零偏慢修正、0.3 死区、互补滤波 0.98/0.02、yaw 积分折 ±180）**和队友的完全一致**，只是把浮点位移的写法改成了显式 `uint16_t/int16_t` 拼接，结果一样。

### 8.4 上电后的执行顺序

1. `main.c` → `MPU6050_Init()`：唤醒 + ±500dps + 开 DWT 计时，并读一次 `WHO_AM_I` 判断在线。
2. 状态机每 10ms 调一次 `MPU6050_Proc()`：读 14 字节 → 去零偏 → 静止检测 → 互补滤波 → yaw 积分。
3. 按键起步（IDLE → RUN）时自动 `MPU6050_ResetYaw()`，yaw 从 0 算起，转向角度才对得上。
4. `turn.c` 的 `BSP_IMU_GetYaw()` 现在返回 `MPU6050_YawData()`（-180 ~ +180 度），`Rota_Update()` 拿它做原地转向。

静止零偏标定（可选，约 2 秒，车必须完全不动）：把 `main.c` 里 `//MPU6050_Calibrate();` 的注释去掉。
`MPU6050_Proc()` 内部本来就会自动学零偏，一般不用手动标。

### 8.5 顺手修掉的两个问题

1. `turn.c` 里 `turn_Update()` 写成了 `Sensor_Find(target_turn, int16_t *pwm_L, int16_t *pwm_R)` —— 非法实参，这个文件参与编译会直接报语法错误。已用 `#if 0 … #endif` 关掉，旁边写了正确写法。
2. 队友 `TIM3_IRQHandler` 和我们 `stm32f1xx_it.c` 里的同名，链接会报重复定义，已删。

### 8.6 怎么验证陀螺仪通了

主循环里临时加一段（500 ms 打一次，免得刷屏）：

```c
static uint32_t t = 0;
if (HAL_GetTick() - t > 500) {
    t = HAL_GetTick();
    UART_SendInt((int16_t)MPU6050_YawData());
    UART_SendStr("\r\n");
}
```

- 手动把车原地转 90°，串口读数应该在 ±90 附近（差几度正常，纯陀螺积分会慢慢漂）。
- `MPU6050_IsOnline()` 返回 0 就是 I2C 没通：先查 PB8/PB9 接线和上拉，再确认 `MPU6050_USE_HW_I2C` 和实际接线一致。
- 车放着不动时 `MPU6050_YawRateData()` 应该接近 0；一直不归零就做一次 `MPU6050_Calibrate()`。

---

## 九、航向环（直行保直 + 按角度转向）

### 9.1 哪几段用上了

| 题目 | 阶段 | 现在的动作 |
|---|---|---|
| 1 | A→B | 航向环保直盲走（15s 超时保护不变） |
| 2 | A→B、C→D | 航向环保直；B→C、D→A 仍是 7 路灰度循迹 |
| 3、4 | A 点、B 点 | 原地转到 `TASK_TURN_A_DEG` / `TASK_TURN_B_DEG`（默认 +90° / -90°），到位自动切直行 |

两个细节：

- **转向期间暂停过点判定**：原地打转时灰度会乱跳，不暂停就会把"打转"误当成一个顶点，阶段直接乱掉。
- **转完自动对齐传感器状态**：`s_last_sensor_state = Sensor_GetState()`，避免下一拍立刻误判一个边沿。

### 9.2 参数全在 `task.h` 的"航向环参数"一节

| 宏 | 默认 | 管什么 |
|---|---|---|
| `TASK_YAW_REVERSE` | 0 | 陀螺仪符号反了/修正方向反了 → 改 1，直行和转向一起纠正 |
| `TASK_HEAD_KP` | 8.0 | 直行：每 1° 偏差给多少 PWM |
| `TASK_HEAD_KD` | 40.0 | 直行阻尼，压左右摆头 |
| `TASK_HEAD_DEADBAND_DEG` | 0.8 | 偏差多小就不修 |
| `TASK_HEAD_MAX_CORR` | 250.0 | 直行差速限幅 |
| `TASK_TURN_KP` | 6.0 | 转向：每 1° 给多少 PWM |
| `TASK_TURN_STOP_DEG` | 1.5 | 误差小于这个角就认为转到位 |
| `TASK_TURN_MAX_PWM` | 500.0 | 转向最大 PWM |
| `TASK_MOTOR_MIN_PWM` | 70.0 | 电机死区补偿（小了轮子不转） |
| `TASK_TURN_TIMEOUT_MS` | 3000 | 转不动也要放行，别卡死 |
| `TASK_TURN_A_DEG` / `TASK_TURN_B_DEG` | +90 / -90 | 第 3、4 题两个转向角 |

调参顺序：**先调 `TASK_TURN_KP`**（转过去不超调、不抖）→ **再调 `TASK_HEAD_KP`**（直行摆幅）→ **最后 `TASK_HEAD_KD`**（收掉摆头）。

### 9.3 现场现象 → 改哪个参数

| 现象 | 怎么改 |
|---|---|
| 直行往一边跑，而且**越修越歪** | `TASK_YAW_REVERSE` 改 1 |
| 直行蛇形摆头 | `TASK_HEAD_KP` 减半；还摆就把 `TASK_HEAD_KD` 减半，最后试 0 |
| 直行修正太肉，偏一大截才回来 | `TASK_HEAD_KP` 8 → 12 |
| 转向转过头（超调） | `TASK_TURN_KP` 6 → 4 |
| 转向慢吞吞、眼看要超时 | `TASK_TURN_KP` 6 → 10 |
| 转向时轮子嗡嗡响但不转 | `TASK_MOTOR_MIN_PWM` 70 → 100 |
| 转向方向反了，但直行是正常的 | `TASK_TURN_A_DEG` / `TASK_TURN_B_DEG` 改符号 |
| 串口报 `Turn TimeOut` 就往下走了 | 加大 `TASK_TURN_TIMEOUT_MS`，同时加大 `TASK_TURN_KP` |

### 9.4 串口会打印什么

```
Start Problem 3      <- 按键起跑，第 3 题
Turn 90 -> 90        <- 开始原地转向（目标航向 90°）
Point=1              <- 识别到一个顶点，阶段切换
Turn TimeOut         <- 3s 还没转到位，放行
P3 Finish            <- 跑完停车
```

### 9.5 一个约定

每个阶段**只让一个东西写马达**：直行段是航向环，循迹段是 `Sensor_Find()`，停车段是 0。
两者不叠加，所以不会打架。以后要"循迹 + 陀螺仪纠偏"叠加，只改 `Task_Motor_Set()` 一个函数就行。

---

## 十、上车前的 5 步台架自检（轮子架空）

按顺序做，每步不通过别往下走。

**1. 按键 → 状态机**
按一下键，串口应打印 `Start Problem 1`，LED 亮、蜂鸣一声。没反应 → `main.h` 里 `KEY_Pin` 到底是 PC15 还是 PB15。

**2. 灰度位图**
把 `main.c` 的 `while(1)` 临时换成：

```c
while (1) {
    uint8_t buf[SENSOR_NUM];
    uint8_t i;
    Sensor_ReadArray(buf);
    for (i = 0; i < SENSOR_NUM; i++) { UART_SendInt(buf[i]); UART_SendStr(i == SENSOR_NUM - 1 ? "\r\n" : " "); }
    UART_SendStr("raw="); UART_SendInt(Sensor_ReadRaw()); UART_SendStr("\r\n");
    HAL_Delay(200);
}
```

手推着车从线的一侧移到另一侧，7 个位应该依次翻。
反向（白线被当成黑线）→ 改 `sensor.h` 的 `SENSOR_BLACK_IS_HIGH`（1↔0）。

**3. 陀螺仪在线**
```c
if (HAL_GetTick() - t > 500) { t = HAL_GetTick();
    UART_SendInt((int16_t)MPU6050_YawData()); UART_SendStr("\r\n"); }
```
手动转 90°，读数应在 ±90 附近。`MPU6050_IsOnline()==0` → 查 PB8/PB9。

**4. 转向方向（最容易错的一步）**
轮子架空，主循环里临时放这段（自包含，不碰状态机）：

```c
/* 轮子架空，主循环里放这一段（自包含，不碰状态机） */
float tgt = MPU6050_YawData() + 90.0f;
Buzzer_ON(); HAL_Delay(300); Buzzer_OFF();
while (1) {
    float e;
    int   out;
    MPU6050_Proc();
    e = tgt - MPU6050_YawData();
    if (e <= 2.0f && e >= -2.0f) { break; }         /* 到位 */
    out = (int)(6.0f * e);
    if (out >  300) { out =  300; }
    if (out < -300) { out = -300; }
    if ((out > 0) && (out < 70))  { out =  70; }    /* 死区补偿 */
    if ((out < 0) && (out > -70)) { out = -70; }
    Motor_Output_L(-out);
    Motor_Output_R(out);
    HAL_Delay(10);
}
Motor_Output_L(0);
Motor_Output_R(0);
```

- 轮子**往正方向（左）转、yaw 涨到 90 停** → 符号对。
- 转向反了 → `task.h` 里 `TASK_YAW_REVERSE` 改 1（直行会一起纠正）。
- 转一半停住 → `TASK_MOTOR_MIN_PWM` 加大。

**5. 直行保直**
航向环保直跑一小段，看两轮是不是"一方加速一方减速"地自己修正。
越修越歪 → `TASK_YAW_REVERSE` 再翻一次；蛇形 → 减 `TASK_HEAD_KP`。

---

## 十一、只测"陀螺仪自转 90°"（按键还没接时用这个）

按键没接 → 状态机永远停在 IDLE，进不去 RUN，自转功能验证不了。
所以单独做了 `turn_test.c/.h`：**阻塞版**，不管按键、不管状态机、不管灰度。

### 11.1 怎么切过去

`main.c` 的 `USER CODE PD` 里有个开关：

```c
#define USE_GYRO_TURN_TEST   1    /* 1 = 只跑自转测试；0 = 正常跑状态机 */
```

设成 **1** 之后：

- `USER CODE 2` 会调 `TurnTest_Init()`：MPU6050 初始化 + 静止标定（约 2s，蜂鸣器一直响）+ yaw 归零
- `while(1)` 里跑 `TurnTest_RunSequence()`：
  **左转 90° → 停 1s → 右转 90°（回到 0）→ 停 1s**，一直循环

参数调好、自转验证通过后，把这个宏改回 **0** 就回到状态机，别的什么都不用动。

### 11.2 串口会打什么

```
TurnTest ready, yaw=0
TurnTo 90 (from 0)
Arrived yaw=89
TurnTo -90 (from 89)
Arrived yaw=0
```

`Arrived yaw` 和目标差 1~2° 是正常的（纯陀螺积分，本来就会漂）。
出现 `Timeout yaw=...` 说明 3 秒没转到位：`TURNTEST_KP` 太小，或 `TURNTEST_MIN_PWM` 太小、电机压根没转。

### 11.3 现象 → 改哪个参数（全在 `turn_test.h`）

| 现象 | 改 |
|---|---|
| 车往右转（应该往左） | `TURNTEST_YAW_REVERSE` 0 → 1 |
| 转过头才停、来回摆 | `TURNTEST_KP` 6 → 4；还摆就把 `TURNTEST_KD` 设 30 |
| 转得慢、快到 3s 才停 | `TURNTEST_KP` 6 → 10 |
| 轮子嗡嗡响但不转 | `TURNTEST_MIN_PWM` 70 → 100 |
| 差好几度就停了 | `TURNTEST_STOP_DEG` 2 → 1（别小于 0.5，会抖） |
| 起步太猛、轮子打滑 | `TURNTEST_MAX_PWM` 400 → 300 |

### 11.4 只想转一次

`while(1)` 里换成：

```c
TurnTest_TurnRel(90.0f, TURNTEST_TIMEOUT_MS);
while (1) { }            /* 停住，看结果 */
```

### 11.5 台架要点

轮子**架空**测：`Motor_Output_L(-out)` / `Motor_Output_R(out)` 是原地转，
落地测会蹭地、车会跑，看不出转了多少度。

### 11.6 状态机和 turn_test 的关系

两者用的是同一套控制律（P + 死区补偿 + 超时），只是：

| | turn_test.c | task.c 的 Task_Heading_Turn() |
|---|---|---|
| 形式 | 阻塞（内部 while） | 非阻塞（每 10ms 一步） |
| 用途 | 台架验证、调参 | 比赛流程里串在阶段中间 |
| 参数 | `TURNTEST_*` | `TASK_TURN_*` |

**调参建议：先在 turn_test 里把 KP / MIN_PWM 调好，再把数值抄到 task.h 的 `TASK_TURN_*`。**

---

## 十二、MPU 装不平/歪着装/竖着装也能用（任意安装角）

### 12.1 问题

MPU6050 是用杜邦线接的，模块吊在车上一头高一头低，没法水平放置。
原来的算法 `yaw_rate = (wy*sr + wz*cr)/cos(pitch)` 是**假设模块水平安装**推出来的：
模块一旦歪了，`cos(pitch)` 变小、`/cp` 越除越大，转同样的角度 yaw 会成倍虚高；
歪到 90°（竖着贴）时 `cp=0.17` 兜底，yaw 直接就是错的。

### 12.2 解决思路（已实现在 `mpu6050.c`）

关键在于：**车原地转向，转轴一定是"世界竖直方向"，而这个方向正好是静止时加速度计测到的那个 1g 向量。**

所以不需要知道模块装成什么样，直接把三轴角速度投影到重力方向上：

```
g = (ax, ay, az) / |a|            单位重力向量，在模块自身坐标系里表示
yaw_rate = wx*gx + wy*gy + wz*gz  绕竖直轴的角速度（度/秒）
yaw += yaw_rate * dt
```

几何上的意义：

- 平放时 `g=(0,0,1)` → `yaw_rate = wz`，和老算法完全一致（**所以 `TURNTEST_YAW_REVERSE` 的含义不变**）
- 竖着贴时 `g=(0,±1,0)` → `yaw_rate = ±wy`，自动改用另一个轴
- 斜面安装时 `g` 是斜的 → 三个轴按比例混合，`wx/wy` 里那些"跟着歪"的转动会被抵掉，只留下绕竖直轴的部分
- 模块上下颠倒也成立（`g=(0,0,-1)`，`yaw_rate=-wz`，符号自动正确）

重力向量用 **0.05 低通**在线估计（`MPU6050_GRAV_LPF`），滤掉车体振动；
`|a|` 不在 0.7~1.3g 之间（急加速、撞车、掉线）就用上一次的值，不会跳变。

### 12.3 开关

`mpu6050.h`：

```c
#define MPU6050_TILT_FREE_YAW   1    /* 1=投影到重力轴(默认，任意安装角)  0=老的水平安装算法 */
```

### 12.4 怎么确认它真的在补偿

`TurnTest_Init()` 会打一行：

```
Mount tilt=37 deg (yaw is tilt-compensated)
```

`37` 就是模块 Z 轴和竖直方向的夹角（0≈平放，90≈竖贴）。这个角**多大都不影响精度**，
只是让你确认一眼装成什么样。然后照常转 90°，看 `Arrived yaw=89` —— 到位就说明补偿生效了。

### 12.5 唯一要注意的：模块要"固定"在车身上

投影法假设**模块相对车体不动**（角度可以是任意值，但不能一直晃）。杜邦线吊着会摆，所以：

- 用双面胶/扎带把模块贴在车壳、电池或一块小板上，**装成什么角度都行**
- 杜邦线留一点余量，别绷紧（绷紧会被车体振动带着拽）
- 别让它自由摆动：摆动频率 2~5Hz，低通滤不掉，yaw 会来回跳

只要贴住了，**水平、竖直、斜着贴都不影响测量**，不用为了它去改结构。

### 12.6 与 pitch/roll 的关系

`MPU6050_PitchData()/RollData()` 还是老算法（互补滤波），歪装时会读到很大的值——**这两个量现在只用于显示**，
yaw 已经完全不用它们了。航向环、转向、turn_test 全部只依赖 `MPU6050_YawData()`。

---

## 十三、串口打 `MPU NOT FOUND` 时怎么查

### 13.1 现象
开机串口打印：

```
===== BOOT =====
Sensor ok
Key ok
LED ok
Buzzer ok
Task ok
MPU NOT FOUND (check PB8/PB9)
...
Mount tilt=173 deg
TurnTo 90 (from 0)
Timeout yaw=0
```

`yaw` 一直是 0、每段都 `Timeout`，说明 **I2C 上一个字节都没读到**（`WHO_AM_I` 读不到 0x68）。
`Mount tilt=173 deg` 是假的：那是老版本读失败时把栈上垃圾当数据算出来的，现在 `GetData()` 读失败会把数据清 0，这个数会变成 0。

### 13.2 本次加的诊断（下载后串口会直接告诉你卡在哪一层）

| 输出 | 含义 |
|---|---|
| `MPU who=0x68 addr=0x68 (online)` | 通了，配置都对 |
| `MPU who=0xFF ... (NOT FOUND)` | 一个字节都没读回来：器件没应答 |
| `MPU who=0x00 ... (NOT FOUND)` | 有应答但读回全 0：多半没上电/上电没复位 |
| `addr=0x69` | AD0 悬空，代码已自动切到 0x69 |
| `I2C bus scan: nothing ACKs` | **物理层没通**（没供电 / 没上拉 / SCL-SDA 接反 / 接错脚 / 线断） |
| `I2C bus scan: 0x68` | 器件在，问题在配置而不是接线 |
| `I2C bus scan: 0x3C` 之类别的地址 | 线接错到别的器件上了（0x3C 是 OLED 常见地址） |
| `still no MPU, err=1234` | 每次探测都在累加失败计数 |

器件不通时代码不再空转 3 次 `Timeout`，而是**每 2 秒重探一次**，通了自动标定开转，不用按复位。

### 13.3 逐项排查（按可能性从高到低）

1. **真上电了吗**：MPU 的 VCC/GND 如果接在电源模块上、开关没开，模块上的 LED 也可能亮——
   那是经 PB8/PB9 的 I2C 上拉电阻 + ESD 二极管"寄生供电"，电压不足、芯片没复位，I2C 一定失败。
   把 VCC/GND 直接接主控 3.3V/GND，**然后断电重上电**。
2. **SCL/SDA 接反**：本工程 I2C1 重映射到 **PB8 = SCL、PB9 = SDA**（`stm32f1xx_hal_msp.c` 里 `__HAL_AFIO_REMAP_I2C1_ENABLE()`）。
   接反的典型表现就是 `nothing ACKs`。对调两根线试一次最快。
3. **有没有上拉电阻**：GY-521 板载 4.7k 上拉到它的 3.3V，够用；裸模块必须外接 **4.7k × 2 到 3.3V**。
4. **别接到电机/编码器脚上**：PB6/PB7 是 TIM4 编码器、PB10/PB11 是 AIN1/AIN2 电机方向脚，
   看起来都"像" I2C 的 PBx，接上去不会动还会干扰电机。
5. **杜邦线**：I2C 线太长（>20cm）或接触不良，100kHz 下也会 NAK。先用手压住线头试。

### 13.4 只想知道通没通，不用改代码
Keil 进 Debug，Watch 1 里加这几个变量：

| 变量 | 期望 |
|---|---|
| `g_dbg_online` | 1 |
| `MPU6050_WhoAmIRaw()` | 0x68 |
| `MPU6050_ErrCount()` | 不增长 |
| `MPU6050_GetAddr7()` | 0x68 或 0x69 |
| `g_dbg_yaw` | 手转车体时在变 |
### 13.5 通了但 `Timeout yaw=0`：怎么分清"传感器坏"和"车没转"

`MPU ok` + `who=0x70` 说明 **I2C 已经完全通了**（0x70 = MPU6500/925x 系，寄存器布局与 MPU6050 兼容，代码不用改）。
`Mount tilt=176 deg` 说明加速度计读回了真实数据（只是模块装反了/倒着贴，投影法会自动补偿，不影响角度）。

那 `yaw` 一直 0 只可能是两种情况，现在每次转向都会自动打印三者区分（每 `TURNTEST_TRACE_MS` = 300ms 一次）：

```
TurnTo 90 (from 0)
  yaw=0.0 rate=0.0 gz=0.0
  yaw=0.0 rate=0.0 gz=0.0
Timeout yaw=0
```

| 打印 | 含义 | 处理 |
|---|---|---|
| `gz` 有几十~几百，`yaw` 在变，但停在 40° 就 `Timeout` | 车在转，只是转不够 | 加大 `TURNTEST_KP`/`TURNTEST_MAX_PWM`，或 `TURNTEST_MIN_PWM` 抬到 120~150 |
| `gz` 有数，`rate` 也是 0 | 解算被"静止检测"冻住了 | 说明抖动太小被当成静止，把 `MPU6050_GRAV_LPF`/静止阈值放宽 |
| `gz` 一直是 0，`yaw` 一直 0 | 陀螺仪没出数据 | 先看 `who`；0x70 是 MPU6500 系，若 `gz` 恒 0 再查是否买到假芯片 |
| 三个都是 0，而且**轮子转但没有车身转动** | **车根本没转** | 车轮架空/打滑/被压住：放到地上，或一边轮子没吃到力 |

最省事的判据（不用改代码）：**把整车拿在手上左右拧一下**，串口 `yaw` 应该跟着变。
想只看数值、不驱动电机：把 `car_driver/turn_test.h` 里 `TURNTEST_MONITOR` 改成 `1`，重新编译。
它每 250ms 打印 `yaw / rate / gz / gx / tilt`，电机全程不动，可以放心手拧。
### 13.6 串口是有线 TTL，车放地上跑就看不到打印怎么办

三种办法，从省事到彻底：

**办法 1：先手拧验证传感器（30 秒，线接着就行）**
`turn_test.h` 里 `TURNTEST_MONITOR` 改 `1` → Rebuild → 烧进去，电机全程不动，你拿在手上把整车左右各拧 90°。
`yaw` 跟着 ±90 走、`gz` 拧的瞬间显示几十~几百 → 陀螺仪、解算、串口整条链路都没问题，剩下的只是电机/机械。

**办法 2：声光播报（已经内置，不用改代码）**
`TURNTEST_REPORT` 默认就是 `1`。拔掉 TTL、车放地上、上电或按复位，靠听声音判断：

| 声/光 | 含义 |
|---|---|
| 蜂鸣器长响约 2s | 正在做静止标定（这时别碰车） |
| 短鸣 1 声 | 这一段开始转 |
| 长鸣 0.3s + LED 常亮 0.8s | **转到位了** |
| 长鸣 0.8s，然后"嘀"n 声 + LED 闪 n 次 | **超时**，实际只转了 n×15°（0 声 = 一点没转） |

**办法 3：带线只转一次**
`turn_test.h` 里 `TURNTEST_ONCE` 改 `1` → 只做一次 +90° 就停住不动，线不会绕死，还能一边看串口。
注意原地转 90° 会拧线一次，反复循环会把 USB 线绕成麻花，所以带线测就开 `TURNTEST_ONCE`。