#include "stepper_motor.h"

// 编译期守卫：SM_TIMER 必须由用户配置定义——全库唯一的定时器句柄，
// 所有电机共享（如 #define SM_TIMER &htim3）。缺失时直接构建失败，
// 避免运行到 SM_Init 才发现句柄无处可寻。
#ifndef SM_TIMER
#error "SM_TIMER must be defined in stepper_motor_config.h (e.g. #define SM_TIMER &htim3)"
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#ifdef SM_USE_EASYLOGGER
#define LOG_TAG "stepper_motor"
#include "elog.h"
#endif

// 电机控制变量集合结构体
typedef struct {
    SM_State_e state; // 电机状态
    SM_StopType_e stop_type; // 电机停止类型
    uint32_t target_steps; // 目标步数
    uint32_t step_cnt; // 电机运行步数计数 CLK翻转2次=1步
    uint8_t toggle_cnt; // CLK翻转计数
    uint8_t speed; // 速度索引
    uint16_t tick_ticks; // CLK翻转间隔（基频tick数），由速度档位换算
    uint16_t tick_cnt; // 运行中递减的基频tick计数，到0翻转CLK并重载
    TickType_t stop_tick; // 停止时间
    uint8_t auto_sleep_disable; // 1=暂停自动休眠（SM_Wake设置，SM_Run清除）
} SM_Vars_t;

typedef struct {
    uint8_t id; // 电机ID
    SM_StopType_e stop_type; // 电机停止类型
} SM_Report_t;

#define SPEED_CURVE_SIZE 10 // 速度档位数(合法档位1~10，内部索引0~9)；须与 SM_SPEED_PERIODS 条目数一致

// 速度表由用户配置的 X 宏 SM_SPEED_PERIODS 提供（见 stepper_motor_config_template.h）。
// 共享定时器以 SM_BASE_TICK_US 固定周期运行，每个电机的实际翻转间隔 =
// 表值 / SM_BASE_TICK_US 个基频tick。此处据该宏生成数组并逐档做编译期整除校验。
#define SM_PERIOD_ENTRY(us) us,
static const uint16_t sm_pulse_period_us[SPEED_CURVE_SIZE] = {SM_SPEED_PERIODS(SM_PERIOD_ENTRY)};
#undef SM_PERIOD_ENTRY

// 编译期校验：每档 CLK 翻转周期必须是 SM_BASE_TICK_US 的整数倍，
// 否则 start_motor_timer 里的整数除法会静默截断，导致速度失真
#define SM_CHECK_DIVISIBLE(us) \
    _Static_assert((us) % SM_BASE_TICK_US == 0, "sm_pulse_period_us entry must be a multiple of SM_BASE_TICK_US");
SM_SPEED_PERIODS(SM_CHECK_DIVISIBLE)
#undef SM_CHECK_DIVISIBLE

// 编译期校验：SM_DEFAULT_SPEED 必须落在合法档位 1~10，否则 sm_vars[].speed 会越界
// 读取 sm_pulse_period_us[]（大小 SPEED_CURVE_SIZE，索引 0~9），造成未定义行为。
// 放在此处可在用户误配时直接构建失败，而非运行时静默越界。
#if (SM_DEFAULT_SPEED < 1) || (SM_DEFAULT_SPEED > SPEED_CURVE_SIZE)
#error "SM_DEFAULT_SPEED must be in range 1..10 (SPEED_CURVE_SIZE)"
#endif

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
 * @brief 判断除指定电机外是否还有 RUNNING 状态的电机
 *
 * 全库只使用一个定时器（见 SM_TIMER）。定时器"运行 ⟺ 至少一个电机
 * RUNNING"这一不变量，由本函数支撑：启动/停止定时器都据此判断，
 * 避免互相干扰。
 *
 * @param id 电机ID（调用方需保证 id < SM_COUNT）
 * @return pdTRUE 存在其他 RUNNING 电机；pdFALSE 其余电机全空闲
 */
static BaseType_t any_other_running(uint8_t id) {
    for (uint8_t j = 0; j < SM_COUNT; j++) {
        if (j != id && sm_vars[j].state == SM_STATE_RUNNING) {
            return pdTRUE;
        }
    }
    return pdFALSE;
}

/**
 * @brief 启动电机（按速度档位换算分频间隔，必要时启动共享定时器）
 *
 * @param id
 */
static void start_motor_timer(uint8_t id) {
    // 按当前速度档位换算翻转间隔（基频tick数），装载分频计数器
    sm_vars[id].tick_ticks = sm_pulse_period_us[sm_vars[id].speed] / SM_BASE_TICK_US;
    sm_vars[id].tick_cnt   = sm_vars[id].tick_ticks;

    // 仅当无其他 RUNNING 电机时才启动共享定时器并清计数器；
    // 定时器已在跑时启动新电机绝不能清计数器，否则会打乱其他电机的时序
    if (!any_other_running(id)) {
        __HAL_TIM_SET_COUNTER(SM_TIMER, 0);
        if (HAL_TIM_Base_Start_IT(SM_TIMER) != HAL_OK) {
#ifdef SM_USE_EASYLOGGER
            log_e("Motor %d timer start fail.", id);
#endif
            return; // 启动失败则不更新状态
        }
    }

    sm_vars[id].state = SM_STATE_RUNNING;
}

/**
 * @brief 将电机复位到确定的空闲状态
 *
 * 统一处理三条停止路径共有的"摆到已知空闲态"动作：CLK 拉低、清步数/分频
 * 计数、置 IDLE、刷新 stop_tick。共享定时器只在已无 RUNNING 电机时才
 * 停止，避免停掉仍在运行的电机。集中于此可避免某条路径遗漏 CLK 复位等
 * 引脚安全操作。
 *
 * @param id     电机ID
 * @param in_isr 是否处于中断上下文（pdTRUE/pdFALSE），用于选择 tick 获取方式
 * @note 本函数不自带临界区，调用方需自行按上下文包好临界区
 */
static void reset_motor_to_idle(uint8_t id, BaseType_t in_isr) {
    // 先把本电机置为 IDLE 并清计数，再判断是否还有 RUNNING 电机，
    // 以便仅当已无电机运行时才停止共享定时器
    sm_vars[id].state = SM_STATE_IDLE;
    HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET); // CLK 复位到确定电平
    sm_vars[id].toggle_cnt = 0;
    sm_vars[id].step_cnt   = 0;
    sm_vars[id].tick_cnt   = 0;

    if (!any_other_running(id)) {
        HAL_TIM_Base_Stop_IT(SM_TIMER);
        __HAL_TIM_SET_COUNTER(SM_TIMER, 0); // 重置计数器，防止下次启动时残留值导致异常
    }

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
static void send_report_isr_aware(uint8_t id, SM_StopType_e stop_type, BaseType_t in_isr) {
    SM_Report_t report = {id, stop_type};

    if (in_isr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(sm_report_queue, &report, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        // 非阻塞发送：队列满则直接丢弃该报告（提示性报告，丢失影响小），避免公共 API 阻塞调用方
        xQueueSend(sm_report_queue, &report, 0);
    }
}

/**
 * @brief 校验电机硬件配置是否有效（三组 GPIO 端口均非空）
 *
 * 用户配置错误（如端口填 NULL）会导致 HAL_GPIO_WritePin/ReadPin 触发 HardFault。
 * 公共 API 在解引用 sm_hw_table[id] 前应先经此校验。
 *
 * @param id 电机ID（调用方需先保证 id < SM_COUNT）
 * @return pdTRUE 配置有效；pdFALSE 存在 NULL 句柄（用户配置错误）
 */
static BaseType_t sm_hw_is_valid(uint8_t id) {
    const SM_HwConfig_t* hw = &sm_hw_table[id];

    return ((hw->sw_port != NULL) && (hw->clk_port != NULL) && (hw->dir_port != NULL)) ? pdTRUE : pdFALSE;
}

/**
 * @brief 在中断中停止步进电机
 *
 * @param id
 */
static void stop_motor_from_isr(uint8_t id) {
    // 用 ISR 临界区包住"停定时器 + 判定 stop_type + 上报"整段，
    // 使其与 SM_StopByLimitISR 的 ISR 路径互斥，避免限位中断在 reset_motor_to_idle
    // 执行中途抢占，导致 stop_type 被改写为 LIMIT 后又重复上报。
    UBaseType_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();

    reset_motor_to_idle(id, pdTRUE);

    if (!sm_hw_table[id].continuous) // 连续运转的电机不发送报告
    {
        // 若 stop_type 已被限位中断置为 LIMIT，则保持，不再改写为 NORMAL
        if (sm_vars[id].stop_type == SM_STOP_NONE) {
            sm_vars[id].stop_type = SM_STOP_NORMAL;
        }

        send_report_isr_aware(id, sm_vars[id].stop_type, pdTRUE);
    }

    taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
}

/**
 * @brief 共享定时器回调（统一处理步数模式与连续模式）
 *
 * 全库只使用一个定时器（见 SM_TIMER），回调遍历所有电机并逐个处理。
 * 每个电机以自身速度档位对应的 tick_ticks 分频翻转 CLK：步数模式计步
 * 并在达标后停止，连续模式仅翻转 CLK。
 *
 * @param htim 触发中断的定时器句柄（未使用，签名由 HAL 回调要求）
 */
static void sm_timer_callback(TIM_HandleTypeDef* htim) {
    UNUSED(htim);

    for (uint8_t id = 0; id < SM_COUNT; id++) {
        // 只处理 RUNNING 电机；IDLE 电机跳过且不停定时器（定时器可能正服务其他电机）
        if (sm_vars[id].state != SM_STATE_RUNNING) {
            continue;
        }

        // 按基频tick分频：递减到0才翻转CLK并重载
        if (--sm_vars[id].tick_cnt > 0) {
            continue;
        }
        sm_vars[id].tick_cnt = sm_vars[id].tick_ticks;

        HAL_GPIO_TogglePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin);

        if (sm_hw_table[id].continuous) {
            continue; // 连续模式不计步，仅由 SM_StopContinuous 停止
        }

        sm_vars[id].toggle_cnt++;
        if (sm_vars[id].toggle_cnt >= 2) {
            sm_vars[id].toggle_cnt = 0;
            sm_vars[id].step_cnt++;

            if (sm_vars[id].step_cnt >= sm_vars[id].target_steps) {
                stop_motor_from_isr(id);
            }
        }
    }
}

/**
 * @brief 轮询所有空闲超时的电机并执行自动休眠
 */
static void sm_auto_sleep_poll(void) {
    const uint32_t now_tick = xTaskGetTickCount();

    for (uint8_t id = 0; id < SM_COUNT; id++) {
        if (sm_hw_table[id].no_sleep || sm_vars[id].auto_sleep_disable) {
            continue;
        }

        // 单字节 volatile 读取本身原子，无需临界区
        if (sm_vars[id].state != SM_STATE_IDLE) {
            continue;
        }

        if (HAL_GPIO_ReadPin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin) != GPIO_PIN_RESET) {
            continue;
        }

        // 无符号减法在tick溢出时仍能正确计算差值，前提是实际间隔 < UINT32_MAX/2（约24.8天）
        // SLEEP_TIMEOUT_MS为3秒，永远不会触发此边界
        if (now_tick - sm_vars[id].stop_tick >= pdMS_TO_TICKS(SLEEP_TIMEOUT_MS)) {
            HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_SET); // 失能电机
            HAL_GPIO_WritePin(sm_hw_table[id].clk_port, sm_hw_table[id].clk_pin, GPIO_PIN_RESET); // CLK拉低
#ifdef SM_USE_EASYLOGGER
            log_i("Motor %d entry sleep.", id);
#endif
        }
    }
}

/**
 * @brief 电机任务入口函数
 * @param para
 */
static void task_entry(void* para) {
    SM_Report_t report;
    for (;;) {
        sm_auto_sleep_poll();

        if (xQueueReceive(sm_report_queue, &report, 0) == pdPASS) {
            SM_ReportAction(report.id, report.stop_type);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 步进电机驱动初始化
 */
void SM_Init(void) {
    for (uint8_t id = 0; id < SM_COUNT; id++) {
        sm_vars[id].state     = SM_STATE_IDLE;
        sm_vars[id].stop_type = SM_STOP_NONE;
        sm_vars[id].speed     = SM_DEFAULT_SPEED - 1;
    }

    // 全库唯一的定时器（SM_TIMER）只注册一次回调、设一次基频周期
    __HAL_TIM_SET_AUTORELOAD(SM_TIMER, SM_BASE_TICK_US);
    if (HAL_TIM_RegisterCallback(SM_TIMER, HAL_TIM_PERIOD_ELAPSED_CB_ID, sm_timer_callback) != HAL_OK) {
        return;
    }

    sm_report_queue =
        xQueueCreateStatic(SM_REPORT_QUEUE_LEN, sizeof(SM_Report_t), sm_report_queue_buf, &sm_report_queue_struct);
    if (sm_report_queue == NULL) {
        return;
    }
    xTaskCreateStatic(task_entry, "sm", SM_TASK_STACK_SIZE, NULL, SM_TASK_PRIORITY, sm_task_stack, &sm_task_struct);
}

/**
 * @brief 启动电机
 *
 * @param id
 * @param dir
 * @param steps
 */
void SM_Run(uint8_t id, uint8_t dir, uint32_t steps) {
    // 仅任务上下文可调用：内部会 vTaskDelay 使能电机，且调用 xQueueSend，
    // 若在中断中调用会导致调度器断言/HardFault。ISR 场景请用 SM_StopByLimitISR 等。
    if (xPortIsInsideInterrupt() || id >= SM_COUNT || !sm_hw_is_valid(id) || dir >= SM_DIR_NUMS || steps == 0) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor run assert fail.");
#endif
        return;
    }

    taskENTER_CRITICAL();
    if (sm_vars[id].state != SM_STATE_IDLE) {
        taskEXIT_CRITICAL();
        SM_Report_t report = {id, SM_STOP_BUSY};

        // 非阻塞发送：队列满则直接丢弃该 BUSY 报告，避免公共 API 阻塞调用方
        xQueueSend(sm_report_queue, &report, 0);
#ifdef SM_USE_EASYLOGGER
        log_w("Motor %d is busy.", id);
#endif
        return;
    }

    sm_vars[id].target_steps       = steps; // 更新目标步数
    sm_vars[id].state              = SM_STATE_READY;
    sm_vars[id].stop_type          = SM_STOP_NONE;
    sm_vars[id].toggle_cnt         = 0;
    sm_vars[id].step_cnt           = 0;
    sm_vars[id].auto_sleep_disable = 0; // 回归正常自动休眠队列

    taskEXIT_CRITICAL();

    // 设置电机方向
    GPIO_PinState set_pin =
        (dir == SM_DIR_FORWARD ? sm_hw_table[id].forward_pin : (GPIO_PinState) (!sm_hw_table[id].forward_pin));
    HAL_GPIO_WritePin(sm_hw_table[id].dir_port, sm_hw_table[id].dir_pin, set_pin);
    // 使能电机
    HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时,留给电机使能的时间

    taskENTER_CRITICAL();
    if (sm_vars[id].state == SM_STATE_READY) // 重新确认
    {
        start_motor_timer(id);
    } else {
        // 使能窗口内被限位停止，立即关闭电机
        HAL_GPIO_WritePin(sm_hw_table[id].sw_port, sm_hw_table[id].sw_pin, GPIO_PIN_SET);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 停止连续运转电机
 */
void SM_StopContinuous(uint8_t id) {
    if (id >= SM_COUNT || !sm_hw_is_valid(id) || !sm_hw_table[id].continuous) {
        return;
    }

    if (xPortIsInsideInterrupt()) {
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
SM_State_e SM_GetState(uint8_t id) {
    if (id >= SM_COUNT) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d get state assert fail.", id);
#endif
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
SM_Dir_e SM_GetDir(uint8_t id) {
    if (id >= SM_COUNT || !sm_hw_is_valid(id)) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d get dir assert fail.", id);
#endif
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
void SM_SetSpeed(uint8_t id, uint8_t speed) {
    if (id >= SM_COUNT || speed > SPEED_CURVE_SIZE || speed == 0) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d set speed assert fail.", id);
#endif
        return;
    }

    uint8_t index     = speed - 1;
    sm_vars[id].speed = index;

    // 更新翻转间隔；运行中的电机在下次翻转重载时生效
    sm_vars[id].tick_ticks = sm_pulse_period_us[index] / SM_BASE_TICK_US;
}

/**
 * 获取电机速度档位
 * @param id 电机ID
 * @return 速度档位
 */
uint8_t SM_GetSpeed(uint8_t id) {
    if (id >= SM_COUNT) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d get speed assert fail.", id);
#endif
        return 0xFF;
    }

    return sm_vars[id].speed + 1;
}
/**
 * @brief 限位定时器中断扫描中硬停电机
 * @param id
 */
void SM_StopByLimitISR(uint8_t id) {
    if (!xPortIsInsideInterrupt()) {
        return;
    }
    if (id >= SM_COUNT || !sm_hw_is_valid(id)) {
        return;
    }
    UBaseType_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
    if (sm_vars[id].state != SM_STATE_IDLE) {
        reset_motor_to_idle(id, pdTRUE);
        sm_vars[id].stop_type = SM_STOP_LIMIT;
    }
    taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
}
/**
 * @brief 限位停止电机的上报接口
 *
 * @param id
 */
void SM_StopReportByLimit(uint8_t id) {
    if (xPortIsInsideInterrupt()) {
        return;
    }

    if (id >= SM_COUNT || !sm_hw_is_valid(id)) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d stop by limit assert fail.", id);
#endif
        return;
    }
    if (sm_vars[id].stop_type == SM_STOP_LIMIT) {
        send_report_isr_aware(id, SM_STOP_LIMIT, pdFALSE);
    }
}

/**
 * @brief 唤醒电机（使能并保持电流，暂停自动休眠）
 * @param id
 */
void SM_Wake(uint8_t id) {
    if (id >= SM_COUNT || !sm_hw_is_valid(id)) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d wake assert fail.", id);
#endif
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
void SM_Sleep(uint8_t id) {
    if (id >= SM_COUNT || !sm_hw_is_valid(id)) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d sleep assert fail.", id);
#endif
        return;
    }

    // 仅 IDLE 状态允许休眠；运转中调用 Sleep 视为误用，直接返回，
    // 避免"驱动已失能但定时器仍在跑、状态机仍 RUNNING"的软硬件失同步。
    if (sm_vars[id].state != SM_STATE_IDLE) {
#ifdef SM_USE_EASYLOGGER
        log_e("Motor %d sleep,motor not idle.", id);
#endif
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
__weak void SM_ReportAction(uint8_t id, SM_StopType_e stop_type) {
    UNUSED(id);
    UNUSED(stop_type);
}
