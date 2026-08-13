# CODEBUDDY.md

This file provides guidance to CodeBuddy Code when working with code in this repository.

## 本仓库是什么

MultiStepMotor 是一个面向 STM32 + FreeRTOS 嵌入式项目的轻量级多步进电机管理库。它**本身不是一个可独立构建的项目**——没有自己的构建系统、测试框架或 lint 配置。它通过嵌入到宿主 STM32CubeMX 工程中来使用（通常是作为 git submodule）。

本仓库内没有任何构建/lint/测试命令。库只在宿主工程中通过 `add_subdirectory(...)` + `target_include_directories(...)` 参与编译。对本仓库的任何改动，都要靠构建宿主 STM32 工程来验证，而不是在本目录里跑命令。

## 文件结构

- `stepper_motor.h` — 公开 API：状态/方向/停止类型枚举、`SM_HwConfig_t` 结构体、函数声明。
- `stepper_motor.c` — 全部实现（FreeRTOS 任务、定时器 ISR 回调、公开函数）。
- `stepper_motor_config_template.h` — 配置模板，含可覆盖的默认值（`SM_DEFAULT_SPEED`、`SLEEP_TIMEOUT_MS`、`SM_TASK_STACK_SIZE`、`SM_TASK_PRIORITY`）。
- `README.md` — 集成指南、配置示例、硬件注意事项（关于用法的权威文档）。

## 配置模型（编辑前必读）

用户**不会**直接编辑本仓库来配置硬件。流程是：

1. 用户把 `stepper_motor_config_template.h` 复制到自己的工程里，重命名为 `stepper_motor_config.h`。
2. 宿主 CMake 的 include 路径顺序把用户的 `Config/` 目录排在**本库目录之前**，于是用户的 `stepper_motor_config.h` 会遮蔽（shadow）模板。
3. `stepper_motor.h` 里 `#include "stepper_motor_config.h"`，依赖的就是这种遮蔽。

用户的配置**必须**（在他们自己的文件里，而不是这里）定义：
- `SM_COUNT` — 电机数量。
- `SM_Id_e` — 电机编号枚举。
- `sm_hw_table[]` — `const SM_HwConfig_t` 数组，按电机 ID 索引（在 `stepper_motor.h` 中声明为 `extern`）。

另有**可选覆盖**项（在模板里给了默认值，用户可在自己的 `stepper_motor_config.h` 里覆盖）：`SM_DEFAULT_SPEED`、`SM_BASE_TICK_US`（共享定时器基频周期 µs）、`SM_SPEED_PERIODS`（各档 CLK 翻转周期的 X 宏列表，条目数须等于 `SPEED_CURVE_SIZE`）、`SLEEP_TIMEOUT_MS`、`SM_TASK_STACK_SIZE`、`SM_TASK_PRIORITY`。

`SM_HwConfig_t` 把每个电机映射到它的 SW/CLK/DIR GPIO 端口+引脚、`forward_pin` 极性、一个 `TIM_HandleTypeDef *`，以及两个行为标志：`continuous`（1 = 连续/泵模式，不因步数停下）和 `no_sleep`（1 = 永不自动休眠，例如必须保持扭矩的垂直轴）。

## 架构（大局观）

**运动由定时器中断驱动，而不是由任务驱动。** 每个电机的 `timer` 运行 `HAL_TIM_Base_Start_IT`；`HAL_TIM_PERIOD_ELAPSED_CB_ID` 回调翻转 CLK 引脚。**CLK 翻转 2 次 = 1 步。**

**定时器共享：** 多个电机可以在 `sm_hw_table[]` 里指向同一个 `TIM_HandleTypeDef *`，共享一个定时器。定时器以固定基频（`SM_BASE_TICK_US`，默认 50µs，用户可在配置里覆盖）运行，每个电机按自己的速度档位分频：`tick_ticks = sm_pulse_period_us[档位] / SM_BASE_TICK_US`，ISR 里递减 `tick_cnt`，减到 0 才翻转一次 CLK 并重载。速度表 `sm_pulse_period_us[SPEED_CURVE_SIZE]` 存的是每个档位的 CLK 翻转周期（µs），由用户配置的 X 宏 `SM_SPEED_PERIODS` 提供，必须是基频的整数倍（10 档合法速度，用户侧档位 1–10 对应数组下标 0–9）。

`SM_Init()` 对每个**唯一**的定时器句柄注册一次统一回调 `sm_group_timer_callback(htim)`（按句柄去重），该回调遍历所有共用此句柄的电机：
- 步数模式：分频翻转 CLK、计步，当 `step_cnt >= target_steps` 时调用 `stop_motor_from_isr()`。
- 连续模式：只分频翻转 CLK；仅通过 `SM_StopContinuous()` 停止。

共享定时器的启停遵守一条不变量：**定时器运行 ⟺ 组内至少一个电机 RUNNING**。`start_motor_timer()` 仅在组内无其他 RUNNING 电机时才 `HAL_TIM_Base_Start_IT`（并清计数器）；`reset_motor_to_idle()` 仅在组内已无 RUNNING 电机时才 `HAL_TIM_Base_Stop_IT`。`group_has_running(id)` 是这条不变量的判定点。

**状态机：** `IDLE → READY → RUNNING → IDLE`。`SM_Run()` 置 READY、使能电机（SW 引脚拉低）、延时约 5ms 等待驱动就绪、重新确认状态仍为 READY，然后启动定时器。`reset_motor_to_idle()` 是所有停止路径（`stop_motor_from_isr`、`SM_StopContinuous`、`SM_StopByLimit`）共同经过的唯一辅助函数，用于把电机摆回已知空闲态——它把 CLK 拉低、清步数/分频计数、置 IDLE、刷新 `stop_tick`；仅当组内已无 RUNNING 电机时才停共享定时器（见上一条不变量）。

**并发模型：** 一个静态分配的 FreeRTOS 任务（`task_entry`，栈和队列全部静态分配——不使用堆）每 10ms 轮询一次：运行 `sm_auto_sleep_poll()` 并取出 `sm_report_queue`，把每条报告派发给 `__weak` 回调 `SM_ReportAction(id, stop_type)`（用户重写它以接收完成/限位/忙事件）。报告要走队列，是因为完成事件起源于 ISR 上下文；`send_report_isr_aware()` 根据上下文选择 `xQueueSendFromISR` 还是 `xQueueSend`。全程临界区都是上下文感知的（`taskENTER_CRITICAL` 对 `taskENTER_CRITICAL_FROM_ISR`）；`SM_StopByLimit` 可能从 EXTI ISR 调用，因此它和 `stop_motor_from_isr` 通过临界区互斥，避免限位中断与正常停止竞争、破坏 `stop_type`。

**自动休眠：** `sm_auto_sleep_poll()` 对任何处于 IDLE 且 SW 引脚仍使能、空闲时间超过 `SLEEP_TIMEOUT_MS` 的电机执行失能（SW 拉高、CLK 拉低）。`no_sleep` 电机以及置了 `auto_sleep_disable`（由 `SM_Wake` 设置；`SM_Run` 清除）的电机跳过。

**上下文限制（改 API 时重要）：** `SM_Run()` 只能在任务上下文调用——它会 `vTaskDelay` 并调用 `xQueueSend`。可从 ISR 调用的入口是 `SM_StopByLimit`（以及内部的定时器回调）。公开 API 在解引用 `sm_hw_table[id]` 之前，都通过 `id >= SM_COUNT` 检查和 `sm_hw_is_valid()` 的空句柄校验来防护。

## 硬件注意事项（来自 README）

- 宿主 STM32CubeMX 中本库所用定时器**必须**设置 **Auto-Reload Preload (ARPE) = Enable** 且 **Period = 0**。CubeMX 默认把 Period 设为最大值；在 ARPE 开启时新的 ARR 要到下一个更新事件才加载，因此 32 位定时器在 72 MHz 下会空转约 59 秒电机才动。
- 每个电机需要 3 个 GPIO（SW = 低电平有效使能、CLK = 脉冲、DIR = 方向）。多个电机可共享 1 个定时器（在 `sm_hw_table[]` 里填同一个句柄）。

## 风格约定

- 注释和文档用中文书写；改动代码注释时保持该约定。
- `sm_vars[]` 保存每个电机的全部运行时状态；有一个编译期 `#error` 守卫强制 `1 <= SM_DEFAULT_SPEED <= SPEED_CURVE_SIZE`。
- 速度表 `sm_pulse_period_us[]` 由用户配置的 X 宏 `SM_SPEED_PERIODS` 生成，并逐档用 `_Static_assert` 校验每档都是 `SM_BASE_TICK_US` 的整数倍（用户改速度值时只改宏列表，数组和校验自动跟随；非整数倍会编译期报错）。
