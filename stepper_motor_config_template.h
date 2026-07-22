#ifndef __STEPPER_MOTOR_CONFIG_H_
#define __STEPPER_MOTOR_CONFIG_H_

#include "main.h"
#include "tim.h"

// 配置模板 — 复制到你的项目中，重命名为 stepper_motor_config.h，然后按需修改
// 通过CMake include路径顺序，用户的 stepper_motor_config.h 会优先于本文件被找到

// 用户必须在自己的配置文件中定义以下内容：
//   SM_COUNT          - 电机数量
//   SM_Id_e           - 电机编号枚举
//   sm_hw_table[]     - 硬件配置表（.c中定义，声明已在stepper_motor.h中提供）

// 以下为可选覆盖的默认值
#ifndef SM_DEFAULT_SPEED
#define SM_DEFAULT_SPEED 2 // 默认速度档位（1~10）
#endif

#ifndef SLEEP_TIMEOUT_MS
#define SLEEP_TIMEOUT_MS 3000 // 电机休眠超时时间(ms)
#endif

#ifndef SM_TASK_STACK_SIZE
#define SM_TASK_STACK_SIZE 512
#endif

#ifndef SM_TASK_PRIORITY
#define SM_TASK_PRIORITY 5
#endif

#endif /* __STEPPER_MOTOR_CONFIG_H_ */
