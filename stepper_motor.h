#ifndef __STEPPER_MOTOR_H_
#define __STEPPER_MOTOR_H_

#include <stdint.h>
#include "stepper_motor_config.h"

// 电机状态
typedef enum {
    SM_STATE_IDLE,
    SM_STATE_READY,
    SM_STATE_RUNNING,
    SM_STATE_NUMS,
} SM_State_e;

// 电机方向
typedef enum {
    SM_DIR_FORWARD,
    SM_DIR_REVERSE,
    SM_DIR_NUMS,
} SM_Dir_e;

// 电机停止类型
typedef enum {
    SM_STOP_NONE,
    SM_STOP_NORMAL, // 到达目标步数
    SM_STOP_LIMIT, // 限位触发停止
    SM_STOP_BUSY, // 电机忙
    SM_STOP_NUMS,
} SM_StopType_e;

// 电机硬件接口结构体
typedef struct {
    GPIO_TypeDef *sw_port;
    uint16_t sw_pin;

    GPIO_TypeDef *clk_port;
    uint16_t clk_pin;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_PinState forward_pin;
    GPIO_PinState reverse_pin;

    TIM_HandleTypeDef *timer;
    uint8_t continuous; // 0=按步数运行, 1=连续运转
    uint8_t no_sleep; // 1=永不自动休眠（如垂直轴需保持电流）
} SM_HwConfig_t;

// 硬件配置表，用户必须在自己的项目中提供定义
extern const SM_HwConfig_t sm_hw_table[];

/**
 * @brief 初始化步进电机驱动（自动注册所有定时器回调，创建任务和队列）
 */
void SM_Init(void);

/**
 * @brief 启动电机运行指定步数
 */
void SM_Run(uint8_t id, uint8_t dir, uint32_t steps);

/**
 * @brief 停止连续运转的电机（仅对continuous=1的电机有效）
 */
void SM_StopContinuous(uint8_t id);

/**
 * @brief 获取电机状态
 */
SM_State_e SM_GetState(uint8_t id);

/**
 * @brief 获取电机当前方向
 */
SM_Dir_e SM_GetDir(uint8_t id);

/**
 * @brief 设置电机速度档位（1~10）
 */
void SM_SetSpeed(uint8_t id, uint8_t speed);

/**
 * @brief 获取电机速度档位
 * @return 速度档位1~10，无效返回0xFF
 */
uint8_t SM_GetSpeed(uint8_t id);

/**
 * @brief 限位触发停止电机（由限位模块调用）
 */
void SM_StopByLimit(uint8_t id);

/**
 * @brief 唤醒电机（使能并保持电流，暂停自动休眠）
 */
void SM_Wake(uint8_t id);

/**
 * @brief 休眠电机（失能并释放电流，恢复自动休眠机制）
 */
void SM_Sleep(uint8_t id);

/**
 * @brief 电机动作完成回调（__weak，用户重写以处理事件）
 */
void SM_ReportAction(uint8_t id, SM_StopType_e stop_type);

#endif /* __STEPPER_MOTOR_H_ */
