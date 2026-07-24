#include "stepper_motor.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// 电机控制变量集合结构体
typedef struct
{
    SM_State_e state;           // 电机状态
    SM_StopType_e stop_type;    // 电机停止类型
    uint32_t target_steps;      // 目标步数
    uint32_t step_cnt;          // 电机运行步数计数 CLK翻转2次=1步
    uint8_t toggle_cnt;         // CLK翻转计数
    uint8_t speed;              // 速度索引
    TickType_t stop_tick;       // 停止时间
    uint8_t auto_sleep_disable; // 1=暂停自动休眠（SM_Wake设置，SM_Run清除）
} SM_Vars_t;

typedef struct
{
    uint8_t id;              // 电机ID
    SM_StopType_e stop_type; // 电机停止类型
} SM_Report_t;

#define SPEED_CURVE_SIZE 10 // 速度档位

// 速度档位对应的定时器中断周期(µs)，即步进脉冲半周期
// 索引0~2: 旧版3档兼容(低速档)；索引3~9: 扩展档位(高速档)
static const uint16_t sm_pulse_period_us[SPEED_CURVE_SIZE] = {150, 300, 450, 50, 75, 100, 125, 175, 200, 225};

static volatile SM_Vars_t sm_vars[SM_COUNT] = {0}; // 所有运行时状态集中管理

// 报告队列容量：每电机至少 1 条完成报告，乘以 2 留余量（忙报告等），
// 避免改为非阻塞发送后瞬时集中上报而丢弃
#define SM_REPORT_QUEUE_LEN (SM_COUNT * 2)

static QueueHandle_t sm_report_queue;
static StaticQueue_t sm_report_queue_struct;
static uint8_t sm_report_queue_buf[SM_REPORT_QUEUE_LEN * sizeof(SM_Report_t)];

static StackType_t sm_task_stack[SM_TASK_STACK_SIZE];
static StaticTask_t sm_task_struct;

/**
 * @brief 启动电机对应的定时器
 *
 * @param id
 * @param period_index
 */
static void start_motor_timer(uint8_t id, uint16_t period_index)
{
    if (sm_hw_table[id].timer == NULL || period_index >= SPEED_CURVE_SIZE)
    {
        return;
    }

    __HAL_TIM_SET_AUTORELOAD(sm_hw_table[id].timer, sm_pulse_period_us[period_index]);
    __HAL_TIM_SET_COUNTER(sm_hw_table[id].timer, 0); // 重置计数器，防止残留值导致首次更新事件异常
    if (HAL_TIM_Base_Start_IT(sm_hw_table[id].timer) != HAL_OK)
    {
        return; // 启动失败则不更新状态
    }

    sm_vars[id].state = SM_STATE_RUNNING;
}

/**
 * @brief 将电机复位到确定的空闲状态
 *
 * 统一处理三条停止路径共有的"摆到已知空闲态"动作：停定时器、清计数器、
 * CLK 拉低、清步数计数、置 IDLE、刷新 stop_tick。集中于此可避免某条路径
 * 遗漏 CLK 复位等引脚安全操作。
 *
 * @param id     电机ID
 * @param in_isr 是否处于中断上下文（pdTRUE/pdFALSE），用于选择 tick 获取方式
 * @note 本函数不自带临界区，调用方需自行按上下文包好临界区
 */
static void reset_motor_to_idle(uint8_t id, BaseType_t in_isr)
{
    HAL_TIM_Base_Stop_IT(sm_hw_table[id].timer);
    __HAL_TIM_SET_COUNTER(sm_hw_table[id].timer, 0); // 重置计数器，防止下次启动时残留值导致异常
    HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET); // CLK 复位到确定电平
    sm_vars[id].toggle_cnt = 0;
    sm_vars[id].step_cnt = 0;
    sm_vars[id].state = SM_STATE_IDLE;
    sm_vars[id].stop_tick = in_isr ? xTaskGetTickCountFromISR() : xTaskGetTickCount();
}

/**
 * @brief 上下文安全的完成事件上报
 *
 * 根据调用上下文选择 FreeRTOS 队列发送 API：中断上下文使用 xQueueSendFromISR
 * + portYIELD_FROM_ISR，任务上下文使用 xQueueSend。
 *
 * @param id        电机ID
 * @param stop_type 停止类型
 * @param in_isr    是否处于中断上下文（pdTRUE/pdFALSE）
 */
static void send_report_isr_aware(uint8_t id, SM_StopType_e stop_type, BaseType_t in_isr)
{
    SM_Report_t report = {id, stop_type};

    if (in_isr)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(sm_report_queue, &report, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        // 非阻塞发送：队列满则直接丢弃该报告（提示性报告，丢失影响小），避免公共 API 阻塞调用方
        xQueueSend(sm_report_queue, &report, 0);
    }
}

/**
 * @brief 校验电机硬件配置是否有效（定时器与三组 GPIO 端口均非空）
 *
 * 用户配置错误（如端口填 NULL）会导致 HAL_GPIO_WritePin/ReadPin 触发 HardFault。
 * 公共 API 在解引用 sm_hw_table[id] 前应先经此校验。
 *
 * @param id 电机ID（调用方需先保证 id < SM_COUNT）
 * @return pdTRUE 配置有效；pdFALSE 存在 NULL 句柄（用户配置错误）
 */
static BaseType_t sm_hw_is_valid(uint8_t id)
{
    const SM_HwConfig_t *hw = &sm_hw_table[id];

    return ((hw->timer != NULL) && (hw->sw_port != NULL) &&
            (hw->clk_port != NULL) && (hw->dir_port != NULL))
               ? pdTRUE
               : pdFALSE;
}

/**
 * @brief 在中断中停止步进电机
 *
 * @param id
 */
static void stop_motor_from_isr(uint8_t id)
{
    reset_motor_to_idle(id, pdTRUE);

    if (!sm_hw_table[id].continuous) // 连续运转的电机不发送报告
    {
        // 如果被设置了，那就是限位已经停止了
        if (sm_vars[id].stop_type == SM_STOP_NONE)
        {
            sm_vars[id].stop_type = SM_STOP_NORMAL;
        }

        send_report_isr_aware(id, sm_vars[id].stop_type, pdTRUE);
    }
}

/**
 * @brief 电机定时器回调函数（按步数运行）
 * @param htim
 */
static void sm_timer_callback(TIM_HandleTypeDef *htim)
{
    uint8_t id;
    for (id = 0; id < SM_COUNT; id++)
    {
        if (sm_hw_table[id].timer != NULL && !sm_hw_table[id].continuous &&
            htim->Instance == sm_hw_table[id].timer->Instance)
        {
            break;
        }
    }
    if (id >= SM_COUNT)
    {
        return;
    }

    // 安全检查：只有电机处于RUNNING状态才处理脉冲
    if (sm_vars[id].state != SM_STATE_RUNNING)
    {
        HAL_TIM_Base_Stop_IT(sm_hw_table[id].timer);
        return;
    }

    HAL_GPIO_TogglePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin);

    sm_vars[id].toggle_cnt++;

    if (sm_vars[id].toggle_cnt >= 2)
    {
        sm_vars[id].toggle_cnt = 0;
        sm_vars[id].step_cnt++;

        if (sm_vars[id].step_cnt >= sm_vars[id].target_steps)
        {
            stop_motor_from_isr(id);
        }
    }
}

/**
 * @brief 连续运转电机专用的定时器回调
 * @param htim
 */
static void sm_timer_continuous_callback(TIM_HandleTypeDef *htim)
{
    uint8_t id;
    for (id = 0; id < SM_COUNT; id++)
    {
        if (sm_hw_table[id].timer != NULL && sm_hw_table[id].continuous &&
            htim->Instance == sm_hw_table[id].timer->Instance)
        {
            break;
        }
    }
    if (id >= SM_COUNT)
    {
        return;
    }

    // 安全检查：只有电机处于RUNNING状态才处理脉冲
    if (sm_vars[id].state != SM_STATE_RUNNING)
    {
        HAL_TIM_Base_Stop_IT(sm_hw_table[id].timer);
        return;
    }

    HAL_GPIO_TogglePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin);
}

/**
 * @brief 延时睡眠电机，防止停不稳
 */
static void sm_delay_sleep_poll(void)
{
    const uint32_t now_tick = xTaskGetTickCount();

    for (uint8_t id = 0; id < SM_COUNT; id++)
    {
        if (sm_hw_table[id].no_sleep || sm_vars[id].auto_sleep_disable)
        {
            continue;
        }

        taskENTER_CRITICAL();
        if (sm_vars[id].state != SM_STATE_IDLE)
        {
            taskEXIT_CRITICAL();
            continue;
        }
        taskEXIT_CRITICAL();

        if (HAL_GPIO_ReadPin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin) != GPIO_PIN_RESET)
        {
            continue;
        }

        // 无符号减法在tick溢出时仍能正确计算差值，前提是实际间隔 < UINT32_MAX/2（约24.8天）
        // SLEEP_TIMEOUT_MS为3秒，永远不会触发此边界
        if (now_tick - sm_vars[id].stop_tick >= pdMS_TO_TICKS(SLEEP_TIMEOUT_MS))
        {
            HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_SET);     // 失能电机
            HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET); // CLK拉低
        }
    }
}

/**
 * @brief 电机任务入口函数
 * @param para
 */
static void task_entry(void *para)
{
    SM_Report_t report;
    for (;;)
    {
        sm_delay_sleep_poll();

        if (xQueueReceive(sm_report_queue, &report, 0) == pdPASS)
        {
            SM_ReportAction(report.id, report.stop_type);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 步进电机驱动初始化
 */
void SM_Init(void)
{
    for (uint8_t id = 0; id < SM_COUNT; id++)
    {
        sm_vars[id].state = SM_STATE_IDLE;
        sm_vars[id].stop_type = SM_STOP_NONE;
        sm_vars[id].speed = SM_DEFAULT_SPEED - 1;

        if (sm_hw_table[id].timer == NULL)
        {
            continue;
        }

        if (sm_hw_table[id].continuous)
        {
            HAL_TIM_RegisterCallback(sm_hw_table[id].timer, HAL_TIM_PERIOD_ELAPSED_CB_ID, sm_timer_continuous_callback);
        }
        else
        {
            HAL_TIM_RegisterCallback(sm_hw_table[id].timer, HAL_TIM_PERIOD_ELAPSED_CB_ID, sm_timer_callback);
        }
    }

    sm_report_queue = xQueueCreateStatic(SM_REPORT_QUEUE_LEN, sizeof(SM_Report_t), sm_report_queue_buf, &sm_report_queue_struct);
    xTaskCreateStatic(task_entry, "sm", SM_TASK_STACK_SIZE, NULL, SM_TASK_PRIORITY, sm_task_stack, &sm_task_struct);
}

/**
 * @brief 启动电机
 *
 * @param id
 * @param dir
 * @param steps
 */
void SM_Run(uint8_t id, uint8_t dir, uint32_t steps)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id) || dir >= SM_DIR_NUMS || steps == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (sm_vars[id].state != SM_STATE_IDLE)
    {
        taskEXIT_CRITICAL();
        SM_Report_t report = {id, SM_STOP_BUSY};

        // 非阻塞发送：队列满则直接丢弃该 BUSY 报告，避免公共 API 阻塞调用方
        xQueueSend(sm_report_queue, &report, 0);
        return;
    }

    sm_vars[id].target_steps = steps; // 更新目标步数
    sm_vars[id].state = SM_STATE_READY;
    sm_vars[id].stop_type = SM_STOP_NONE;
    sm_vars[id].toggle_cnt = 0;
    sm_vars[id].step_cnt = 0;
    sm_vars[id].auto_sleep_disable = 0; // 回归正常自动休眠队列

    taskEXIT_CRITICAL();

    // 设置电机方向
    GPIO_PinState set_pin =
        (dir == SM_DIR_FORWARD ? sm_hw_table[id].forward_pin : (GPIO_PinState)(!sm_hw_table[id].forward_pin));
    HAL_GPIO_WritePin(sm_hw_table[id].dir_port, sm_hw_table[id].dir_pin, set_pin);
    // 使能电机
    HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时,留给电机使能的时间

    taskENTER_CRITICAL();
    if (sm_vars[id].state == SM_STATE_READY) // 重新确认
    {
        start_motor_timer(id, sm_vars[id].speed);
    }
    else
    {
        // 使能窗口内被限位停止，立即关闭电机
        HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_SET);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 停止连续运转电机
 */
void SM_StopContinuous(uint8_t id)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id) || !sm_hw_table[id].continuous)
    {
        return;
    }

    if (xPortIsInsideInterrupt())
    {
        return;
    }

    taskENTER_CRITICAL();
    reset_motor_to_idle(id, pdFALSE); // 连续电机停止不发送报告
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取电机状态
 *
 * @param id
 * @return SM_State_e
 */
SM_State_e SM_GetState(uint8_t id)
{
    if (id >= SM_COUNT)
    {
        return SM_STATE_INVALID;
    }
    return sm_vars[id].state;
}

/**
 * @brief 获取电机当前方向
 *
 * @param id
 * @return SM_Dir_e
 */
SM_Dir_e SM_GetDir(uint8_t id)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id))
    {
        return SM_DIR_INVALID;
    }

    GPIO_PinState current_state = HAL_GPIO_ReadPin(sm_hw_table[id].dir_port, sm_hw_table[id].dir_pin);

    return (current_state == sm_hw_table[id].forward_pin ? SM_DIR_FORWARD : SM_DIR_REVERSE);
}

/**
 * @brief 设置电机速度档位
 * @param id 电机ID
 * @param speed 合法值1~10
 */
void SM_SetSpeed(uint8_t id, uint8_t speed)
{
    if (id >= SM_COUNT || speed > SPEED_CURVE_SIZE || speed == 0)
    {
        return;
    }

    uint8_t index = speed - 1;
    sm_vars[id].speed = index;

    // 运行中电机立即更新定时器周期，下一个中断生效
    if (sm_vars[id].state == SM_STATE_RUNNING)
    {
        __HAL_TIM_SET_AUTORELOAD(sm_hw_table[id].timer, sm_pulse_period_us[index]);
    }
}

/**
 * 获取电机速度档位
 * @param id 电机ID
 * @return 速度档位
 */
uint8_t SM_GetSpeed(uint8_t id)
{
    if (id >= SM_COUNT)
    {
        return 0xFF;
    }

    return sm_vars[id].speed + 1;
}

/**
 * @brief 限位停止电机的接口
 *
 * @param id
 */
void SM_StopByLimit(uint8_t id)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id))
    {
        return;
    }

    BaseType_t need_report = pdFALSE;
    BaseType_t in_isr = xPortIsInsideInterrupt();
    UBaseType_t saved_interrupt_status;

    // 限位可能由 EXTI 中断调用，需按上下文选择临界区，使停止在 ISR 与任务两种上下文都安全
    if (in_isr)
    {
        saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
    }
    else
    {
        taskENTER_CRITICAL();
    }

    if (sm_vars[id].state != SM_STATE_IDLE)
    {
        reset_motor_to_idle(id, in_isr);
        sm_vars[id].stop_type = SM_STOP_LIMIT;
        need_report = pdTRUE;
    }

    if (in_isr)
    {
        taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
    }
    else
    {
        taskEXIT_CRITICAL();
    }

    if (need_report)
    {
        send_report_isr_aware(id, SM_STOP_LIMIT, in_isr);
    }
}

/**
 * @brief 唤醒电机（使能并保持电流，暂停自动休眠）
 * @param id
 */
void SM_Wake(uint8_t id)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id))
    {
        return;
    }

    sm_vars[id].auto_sleep_disable = 1;
    HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_RESET);
    sm_vars[id].stop_tick = xTaskGetTickCount();
}

/**
 * @brief 休眠电机（失能并释放电流，恢复自动休眠机制）
 * @param id
 */
void SM_Sleep(uint8_t id)
{
    if (id >= SM_COUNT || !sm_hw_is_valid(id))
    {
        return;
    }

    sm_vars[id].auto_sleep_disable = 0;
    HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET);
}

/**
 * @brief 电机上报动作
 * @param id
 * @param stop_type
 * @note __weak，用户可重写
 */
__weak void SM_ReportAction(uint8_t id, SM_StopType_e stop_type)
{
    UNUSED(id);
    UNUSED(stop_type);
}
