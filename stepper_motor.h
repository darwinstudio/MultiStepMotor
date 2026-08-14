#ifndef __STEPPER_MOTOR_H_
#define __STEPPER_MOTOR_H_

#include "stepper_motor_config.h"
#include <stdint.h>

// 电机状态
typedef enum {
    SM_STATE_IDLE,
    SM_STATE_READY,
    SM_STATE_RUNNING,
    SM_STATE_INVALID, // 无效ID等错误返回
    SM_STATE_NUMS,
} SM_State_e;

// 电机方向
typedef enum {
    SM_DIR_FORWARD,
    SM_DIR_REVERSE,
    SM_DIR_INVALID, // 无效ID等错误返回
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
    GPIO_TypeDef* sw_port;
    uint16_t sw_pin;

    GPIO_TypeDef* clk_port;
    uint16_t clk_pin;

    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    GPIO_PinState forward_pin;

    uint8_t continuous; // 0=按步数运行, 1=连续运转
    uint8_t no_sleep; // 1=永不自动休眠（如垂直轴需保持电流）
} SM_HwConfig_t;

// 硬件配置表，用户必须在自己的项目中提供定义
extern const SM_HwConfig_t sm_hw_table[];

void SM_Init(void);
void SM_Run(uint8_t id, uint8_t dir, uint32_t steps);
void SM_StopContinuous(uint8_t id);
SM_State_e SM_GetState(uint8_t id);
SM_Dir_e SM_GetDir(uint8_t id);
void SM_SetSpeed(uint8_t id, uint8_t speed);
uint8_t SM_GetSpeed(uint8_t id);
void SM_StopByLimitISR(uint8_t id);
void SM_StopReportByLimit(uint8_t id);
void SM_Wake(uint8_t id);
void SM_Sleep(uint8_t id);
void SM_ReportAction(uint8_t id, SM_StopType_e stop_type);

#endif /* __STEPPER_MOTOR_H_ */
