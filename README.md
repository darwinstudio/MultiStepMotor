# MultiStepMotor

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Language](https://img.shields.io/badge/language-C-blue.svg)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green.svg)

一个面向嵌入式系统的轻量级多步进电机管理库，基于 FreeRTOS。

通过定时器中断驱动脉冲输出，支持按步数运行和连续运转两种模式，内置自动休眠和限位停止机制。

## 特性

- 最多支持 N 个步进电机同时管理（数量由用户配置决定）
- 定时器 ISR 驱动，精确脉冲输出
- 10 档速度可调（50μs ~ 450μs 定时器周期）
- 电机空闲 3 秒自动休眠，降低功耗
- 限位触发立即停止，带消抖和报告机制
- 支持按步数运行和连续运转（如蠕动泵）
- 完成事件通过 `__weak` 回调通知应用层
- 与硬件 GPIO/Timer 解耦，通过配置表适配不同项目

## 文件结构

```
MultiStepMotor/
└── stepper_motor/
    ├── stepper_motor.h          # 公开 API
    ├── stepper_motor.c          # 通用实现
    └── stepper_motor_config.h   # 默认配置（用户覆盖）
```

## 集成方式

### 1. 添加 Git Submodule

```bash
git submodule add git@github.com:darwinstudio/MultiStepMotor.git Middlewares/MultiStepMotor
```

### 2. 创建用户配置文件

在你的项目中创建 `stepper_motor_config.h`，通过 CMake include 路径顺序覆盖库内的默认配置：

```
your_project/
├── Config/
│   └── stepper_motor_config.h   # 你的硬件配置（优先级高于库的默认配置）
├── Middlewares/
│   └── MultiStepMotor/          # submodule
└── CMakeLists.txt
```

**CMakeLists.txt:**

```cmake
add_subdirectory(Middlewares/MultiStepMotor/stepper_motor)

target_include_directories(my_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Config                              # 排在前面，覆盖库内同名文件
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/MultiStepMotor/stepper_motor
)
```

### 3. 用户配置文件示例

```c
// stepper_motor_config.h
#ifndef __STEPPER_MOTOR_CONFIG_H_
#define __STEPPER_MOTOR_CONFIG_H_

#include "main.h"
#include "tim.h"

// 软件配置（可选覆盖）
#define SM_DEFAULT_SPEED    2       // 默认速度档位（1~10）
#define SLEEP_TIMEOUT_MS    3000    // 电机休眠超时时间(ms)

// 电机数量
#define SM_COUNT 3

// 电机编号
typedef enum
{
    SM_ID_X_AXIS,
    SM_ID_Y_AXIS,
    SM_ID_PUMP,
} SM_Id_e;

// 硬件配置表声明
extern const SM_HwConfig_t sm_hw_table[SM_COUNT];

#endif
```

### 4. 硬件配置表

```c
// stepper_motor_config.c
#include "stepper_motor_config.h"

const SM_HwConfig_t sm_hw_table[SM_COUNT] = {
    [SM_ID_X_AXIS] = {
        .sw_port = MOTOR_SW_X_GPIO_Port,  .sw_pin = MOTOR_SW_X_Pin,
        .clk_port = MOTOR_CLK_X_GPIO_Port, .clk_pin = MOTOR_CLK_X_Pin,
        .dir_port = MOTOR_DIR_X_GPIO_Port, .dir_pin = MOTOR_DIR_X_Pin,
        .forward_pin = GPIO_PIN_RESET, .reverse_pin = GPIO_PIN_SET,
        .timer = &htim3, .continuous = 0,
    },
    [SM_ID_Y_AXIS] = {
        .sw_port = MOTOR_SW_Y_GPIO_Port,  .sw_pin = MOTOR_SW_Y_Pin,
        .clk_port = MOTOR_CLK_Y_GPIO_Port, .clk_pin = MOTOR_CLK_Y_Pin,
        .dir_port = MOTOR_DIR_Y_GPIO_Port, .dir_pin = MOTOR_DIR_Y_Pin,
        .forward_pin = GPIO_PIN_RESET, .reverse_pin = GPIO_PIN_SET,
        .timer = &htim4, .continuous = 0,
    },
    [SM_ID_PUMP] = {
        .sw_port = MOTOR_SW_P_GPIO_Port,  .sw_pin = MOTOR_SW_P_Pin,
        .clk_port = MOTOR_CLK_P_GPIO_Port, .clk_pin = MOTOR_CLK_P_Pin,
        .dir_port = MOTOR_DIR_P_GPIO_Port, .dir_pin = MOTOR_DIR_P_Pin,
        .forward_pin = GPIO_PIN_SET, .reverse_pin = GPIO_PIN_RESET,
        .timer = &htim5, .continuous = 1,  // 连续运转模式
    },
};
```

### 5. 使用

```c
#include "stepper_motor.h"

// 重写电机完成回调
void SM_ReportAction(uint8_t id, SM_StopType_e stop_type)
{
    if (stop_type == SM_STOP_NORMAL)
    {
        // 电机到达目标位置
    }
    else if (stop_type == SM_STOP_LIMIT)
    {
        // 限位触发停止
    }
}

// 初始化（自动注册定时器回调，创建任务）
SM_Init();

// 启动电机：X轴正转1000步
SM_Run(SM_ID_X_AXIS, SM_DIR_FORWARD, 1000);

// 启动连续运转的泵
SM_Run(SM_ID_PUMP, SM_DIR_FORWARD, 1);

// 停止泵
SM_StopContinuous(SM_ID_PUMP);

// 调整速度（1~10）
SM_SetSpeed(SM_ID_X_AXIS, 5);
```

## 可覆盖的配置项

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `SM_DEFAULT_SPEED` | 2 | 默认速度档位（1~10） |
| `SLEEP_TIMEOUT_MS` | 3000 | 电机休眠超时时间(ms) |
| `SM_TASK_STACK_SIZE` | 512 | 电机任务栈大小(字节) |
| `SM_TASK_PRIORITY` | 5 | 电机任务优先级 |

## API

| 函数 | 说明 |
|------|------|
| `SM_Init()` | 初始化驱动（注册定时器回调，创建任务和队列） |
| `SM_Run(id, dir, steps)` | 启动电机运行指定步数 |
| `SM_StopContinuous(id)` | 停止连续运转的电机 |
| `SM_GetState(id)` | 获取电机状态（IDLE/READY/RUNNING） |
| `SM_GetDir(id)` | 获取电机当前方向 |
| `SM_SetSpeed(id, speed)` | 设置速度档位（1~10），运行中立即生效 |
| `SM_GetSpeed(id)` | 获取当前速度档位 |
| `SM_StopByLimit(id)` | 限位触发停止（由限位模块调用） |
| `SM_ReportAction(id, stop_type)` | 完成回调（`__weak`，用户重写） |

## 硬件接线要求

每个步进电机需要 3 组 GPIO + 1 个定时器：

| 信号 | 说明 |
|------|------|
| SW | 使能引脚（低电平有效） |
| CLK | 脉冲引脚（定时器中断翻转） |
| DIR | 方向引脚 |

## STM32CubeMX 定时器配置注意事项

使用本库时，必须将电机所用定时器的 **Period 设为 0**。

STM32CubeMX 默认将 Period 设为最大值（16 位定时器为 65535，32 位定时器为 4294967295）。由于定时器开启了 Auto-Reload Preload（ARPE），代码中通过 `HAL_TIM` 设置的 ARR 值要到**下一个更新事件**才生效。如果 Period 保持默认最大值，定时器首次运行需要先计数到最大值才会触发更新事件加载新周期——32 位定时器在 72MHz 下需要约 59 秒，表现为电机启动后长时间无动作。

将 Period 设为 0 后，`SM_Init` 中 `__HAL_TIM_SET_AUTORELOAD` 写入的值会在第一个计数周期即生效。

## 依赖

- STM32 HAL 库（GPIO、TIM）
- FreeRTOS（Task、Queue）

## License

[MIT](LICENSE)
