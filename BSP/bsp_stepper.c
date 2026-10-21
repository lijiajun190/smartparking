#include "bsp_stepper.h"
#include "safety.h"
#include "FreeRTOS.h"
#include "task.h"
#include "health_watch.h"

/* 28BYJ-48步进电机特性：
 * - 5线单极步进电机（4个控制线 + 1个公共线）
 * - 减速比：1:64
 * - 步进角：5.625° (内部电机)
 * - 一圈步数：4096步(半步模式) 或 2048步(全步模式)
 */

/* 全步序列表：4步为一个完整周期（四相四拍）
 * 特点：扭矩大，功耗高，运行略有顿挫
 * 每行代表一个相位，四列分别对应 IN1, IN2, IN3, IN4
 */
static const uint8_t full_step_sequence[4][4] = {
    {1, 0, 0, 1},  // 相位0：A相正 + D相正
    {1, 1, 0, 0},  // 相位1：A相正 + B相正
    {0, 1, 1, 0},  // 相位2：B相正 + C相正
    {0, 0, 1, 1}   // 相位3：C相正 + D相正
};

/* 半步序列表：8步为一个完整周期（四相八拍）
 * 特点：精度高一倍，运行平滑，功耗适中
 * 交替使用单相励磁和双相励磁
 */
static const uint8_t half_step_sequence[8][4] = {
    {1, 0, 0, 0},  // 相位0：仅A相
    {1, 1, 0, 0},  // 相位1：A相 + B相
    {0, 1, 0, 0},  // 相位2：仅B相
    {0, 1, 1, 0},  // 相位3：B相 + C相
    {0, 0, 1, 0},  // 相位4：仅C相
    {0, 0, 1, 1},  // 相位5：C相 + D相
    {0, 0, 0, 1},  // 相位6：仅D相
    {1, 0, 0, 1}   // 相位7：D相 + A相
};

/* 电机A和电机B的硬件配置
 * 根据实际接线修改这些引脚定义
 */
static MotorConfig motor_configs[MOTOR_COUNT] = {
    // MOTOR_A配置（水平移动，第一个ULN2003）
    {
        .in1_port = MOTOR_A_IN1_GPIO_Port,
        .in1_pin  = MOTOR_A_IN1_Pin,
        .in2_port = MOTOR_A_IN2_GPIO_Port,
        .in2_pin  = MOTOR_A_IN2_Pin,
        .in3_port = MOTOR_A_IN3_GPIO_Port,
        .in3_pin  = MOTOR_A_IN3_Pin,
        .in4_port = MOTOR_A_IN4_GPIO_Port,
        .in4_pin  = MOTOR_A_IN4_Pin,
        .step_delay_us = PARK_STEP_DELAY_US,
        .max_steps = PARK_AXIS_MAX_STEPS,
        .mode = STEP_HALF       // 默认使用半步模式
    },
    // MOTOR_B配置（垂直移动，第二个ULN2003）
    {
        .in1_port = MOTOR_B_IN1_GPIO_Port,
        .in1_pin  = MOTOR_B_IN1_Pin,
        .in2_port = MOTOR_B_IN2_GPIO_Port,
        .in2_pin  = MOTOR_B_IN2_Pin,
        .in3_port = MOTOR_B_IN3_GPIO_Port,
        .in3_pin  = MOTOR_B_IN3_Pin,
        .in4_port = MOTOR_B_IN4_GPIO_Port,
        .in4_pin  = MOTOR_B_IN4_Pin,
        .step_delay_us = PARK_STEP_DELAY_US,
        .max_steps = PARK_AXIS_MAX_STEPS,
        .mode = STEP_HALF
    }
};

/* 电机运行状态 */
static MotorState motor_states[MOTOR_COUNT];

/* 私有函数：设置电机某一相位的输出状态
 * motor: 电机编号
 * phase: 相位索引（0-3全步 或 0-7半步）
 */
static void stepper_set_phase(MotorID motor, uint8_t phase)
{
    MotorConfig* cfg = &motor_configs[motor];
    uint8_t in1, in2, in3, in4;
    
    // 根据步进模式选择相应的时序表
    if (cfg->mode == STEP_FULL) {
        in1 = full_step_sequence[phase][0];
        in2 = full_step_sequence[phase][1];
        in3 = full_step_sequence[phase][2];
        in4 = full_step_sequence[phase][3];
    } else {
        in1 = half_step_sequence[phase][0];
        in2 = half_step_sequence[phase][1];
        in3 = half_step_sequence[phase][2];
        in4 = half_step_sequence[phase][3];
    }
    
    // 输出到对应的GPIO引脚
    HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in3_port, cfg->in3_pin, in3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in4_port, cfg->in4_pin, in4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 停止请求跨任务/中断共享。中断只锁存，运动任务独占相位输出。 */
static volatile MotorFault stop_fault;
static volatile uint8_t homed[MOTOR_COUNT];
static volatile MotorID active_motor = MOTOR_COUNT;
static volatile uint8_t home_seeking;
static volatile uint8_t home_contact;

static uint8_t valid_motor(MotorID motor)
{
    return (unsigned)motor < MOTOR_COUNT;
}

static uint8_t limit_active(MotorID motor)
{
    return sensor_read(motor == MOTOR_A ? PARK_AXIS_A_LIMIT : PARK_AXIS_B_LIMIT);
}

static void coils_off(MotorID motor)
{
    MotorConfig *cfg = &motor_configs[motor];
    HAL_GPIO_WritePin(cfg->in1_port, cfg->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in2_port, cfg->in2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in3_port, cfg->in3_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->in4_port, cfg->in4_pin, GPIO_PIN_RESET);
}

void stepper_request_stop(MotorFault fault)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (stop_fault == MOTOR_FAULT_NONE)
        stop_fault = fault == MOTOR_FAULT_NONE ? MOTOR_FAULT_STOP : fault;
    homed[MOTOR_A] = homed[MOTOR_B] = 0U;
    __set_PRIMASK(mask);
}

MotorFault stepper_get_fault(void)
{
    return stop_fault;
}

uint8_t stepper_is_homed(MotorID motor)
{
    return valid_motor(motor) && homed[motor] && stop_fault == MOTOR_FAULT_NONE;
}

/* 仅运动任务或确认无在途命令的恢复路径调用。释放急停不会自动解锁。 */
uint8_t stepper_clear_fault(void)
{
    uint8_t cleared = 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (active_motor == MOTOR_COUNT && !sensor_emergency_active())
    {
        stop_fault = MOTOR_FAULT_NONE;
        homed[MOTOR_A] = homed[MOTOR_B] = 0U;
        cleared = 1U;
    }
    __set_PRIMASK(mask);
    return cleared;
}

void stepper_service_stop(void)
{
    if (stop_fault != MOTOR_FAULT_NONE)
    {
        MotorID motor;
        for (motor = MOTOR_A; motor < MOTOR_COUNT; motor++)
        {
            coils_off(motor);
            motor_states[motor].status =
                stop_fault == MOTOR_FAULT_STOP ? MOTOR_STOPPED : MOTOR_FAULT;
        }
    }
}

/* EXTI 捕获短急停和限位触发；寻找原点时触限是预期事件。 */
void stepper_sensor_irq(void)
{
    if (sensor_emergency_active())
        stepper_request_stop(MOTOR_FAULT_EMERGENCY);
    if (active_motor < MOTOR_COUNT &&
        motor_states[active_motor].direction == PARK_HOME_DIRECTION &&
        limit_active(active_motor))
    {
        if (home_seeking)
            home_contact = 1U;
        else
            stepper_request_stop(MOTOR_FAULT_LIMIT);
    }
}

static uint8_t can_continue(MotorID motor, MotorDirection dir, uint8_t seek)
{
    SafetyStatus safety = safety_check_motor(motor, dir);
    if (safety == SAFETY_FAULT_EMERGENCY)
        stepper_request_stop(MOTOR_FAULT_EMERGENCY);
    else if (safety == SAFETY_FAULT_LIMIT)
    {
        if (seek)
            home_contact = 1U;
        else
            stepper_request_stop(MOTOR_FAULT_LIMIT);
    }
    else if (safety != SAFETY_OK)
        stepper_request_stop(MOTOR_FAULT_COMMAND);
    return stop_fault == MOTOR_FAULT_NONE && !(seek && home_contact);
}

/* 每 <=100us 检查停止、方向限位和命令超时，避免完整单步延时后才响应。 */
static uint8_t checked_delay(MotorID motor, MotorDirection dir, uint8_t seek,
                             uint32_t us, uint32_t started)
{
    while (us != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - started) >= STEPPER_COMMAND_TIMEOUT_MS)
            stepper_request_stop(MOTOR_FAULT_TIMEOUT);
        if (!can_continue(motor, dir, seek))
            return 0U;
        uint32_t slice = us > 100U ? 100U : us;
        uint32_t start = DWT->CYCCNT;
        uint32_t cycles = (SystemCoreClock / 1000000U) * slice;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles) {}
        us -= slice;
    }
    return can_continue(motor, dir, seek);
}

static uint8_t one_step(MotorID motor, MotorDirection dir, uint8_t seek,
                        uint16_t delay, uint32_t started)
{
    MotorState *state = &motor_states[motor];
    uint8_t count = motor_configs[motor].mode == STEP_FULL ? 4U : 8U;
    /* 检查与4路相位输出作为短原子段，防止停止中断后再重新励磁。 */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    state->direction = dir;
    if ((uint32_t)(HAL_GetTick() - started) >= STEPPER_COMMAND_TIMEOUT_MS)
        stepper_request_stop(MOTOR_FAULT_TIMEOUT);
    if (!can_continue(motor, dir, seek))
    {
        __set_PRIMASK(mask);
        return 0U;
    }
    state->current_phase = dir == DIR_CW ?
        (state->current_phase + 1U) % count :
        (state->current_phase + count - 1U) % count;
    stepper_set_phase(motor, state->current_phase);
    state->step_count += dir == PARK_AWAY_DIRECTION ? 1 : -1;
    __set_PRIMASK(mask);
    if (!checked_delay(motor, dir, seek, delay, started))
        return 0U;
    /* 让输入/业务任务可运行；每步额外让出一个RTOS tick。 */
    health_beat(HEALTH_MOTION, HAL_GetTick());
    vTaskDelay(1U);
    return can_continue(motor, dir, seek);
}

static uint8_t begin_motion(MotorID motor)
{
    if (!valid_motor(motor))
    {
        stepper_request_stop(MOTOR_FAULT_COMMAND);
        return 0U;
    }
    if (sensor_emergency_active())
        stepper_request_stop(MOTOR_FAULT_EMERGENCY);
    if (stop_fault != MOTOR_FAULT_NONE)
    {
        stepper_service_stop();
        return 0U;
    }
    motor_states[motor].direction = PARK_AWAY_DIRECTION;
    active_motor = motor;
    motor_states[motor].status = MOTOR_RUNNING;
    return 1U;
}

static void finish_motion(MotorID motor)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    home_seeking = 0U;
    if (stop_fault == MOTOR_FAULT_NONE)
        motor_states[motor].status = MOTOR_IDLE;
    active_motor = MOTOR_COUNT;
    __set_PRIMASK(mask);
    stepper_service_stop();
}

void stepper_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;
    stop_fault = MOTOR_FAULT_NONE;
    active_motor = MOTOR_COUNT;
    home_seeking = home_contact = 0U;
    MotorID motor;
    for (motor = MOTOR_A; motor < MOTOR_COUNT; motor++)
    {
        motor_states[motor] = (MotorState){ .status = MOTOR_IDLE };
        homed[motor] = 0U;
        coils_off(motor);
    }
    if (sensor_emergency_active())
        stepper_request_stop(MOTOR_FAULT_EMERGENCY);
}

void stepper_move_steps(MotorID motor, uint32_t steps, MotorDirection dir)
{
    if (!begin_motion(motor))
        return;
    MotorConfig *cfg = &motor_configs[motor];
    MotorState *state = &motor_states[motor];
    uint32_t started = HAL_GetTick();
    if (dir != DIR_CW && dir != DIR_CCW)
        stepper_request_stop(MOTOR_FAULT_COMMAND);
    else if (!homed[motor])
        stepper_request_stop(MOTOR_FAULT_NOT_HOMED);
    else if (steps > cfg->max_steps ||
             (dir == PARK_AWAY_DIRECTION && steps > cfg->max_steps - (uint32_t)state->step_count) ||
             (dir == PARK_HOME_DIRECTION && steps > (uint32_t)state->step_count))
        stepper_request_stop(MOTOR_FAULT_TRAVEL);
    else
    {
        state->direction = dir;
        uint32_t i;
        for (i = 0U; i < steps; i++)
            if (!one_step(motor, dir, 0U, cfg->step_delay_us, started))
                break;
    }
    finish_motion(motor);
}

void stepper_stop(MotorID motor)
{
    if (valid_motor(motor))
        stepper_request_stop(MOTOR_FAULT_STOP);
    /* 由 MotionTask 调用；其他任务/ISR 使用 request_stop。 */
    stepper_service_stop();
}

void stepper_home(MotorID motor)
{
    if (!begin_motion(motor))
        return;
    uint32_t started = HAL_GetTick();
    uint32_t steps = 0U;
    uint8_t found = 0U;
    homed[motor] = 0U;
    /* 上电已压住限位时先退出，再重新寻找，检查开关卡死。 */
    while (limit_active(motor) && steps < STEPPER_HOME_RELEASE_STEPS)
    {
        steps++;
        if (!one_step(motor, PARK_AWAY_DIRECTION, 0U, STEPPER_HOME_DELAY_US, started))
            goto finished;
    }
    if (limit_active(motor))
    {
        stepper_request_stop(MOTOR_FAULT_HOME_STUCK);
        goto finished;
    }
    home_contact = 0U;
    home_seeking = 1U;
    motor_states[motor].direction = PARK_HOME_DIRECTION;
    for (steps = 0U; steps <= STEPPER_HOME_SEEK_STEPS; )
    {
        if (home_contact || limit_active(motor))
        {
            coils_off(motor);
            /* 触限立即停步；静止确认5ms，抖动则继续寻找。 */
            if (!checked_delay(motor, PARK_AWAY_DIRECTION, 0U, 5000U, started))
                goto finished;
            if (limit_active(motor))
            {
                found = 1U;
                break;
            }
            home_contact = 0U;
        }
        if (steps == STEPPER_HOME_SEEK_STEPS)
            break;
        steps++;
        if (!one_step(motor, PARK_HOME_DIRECTION, 1U, STEPPER_HOME_DELAY_US, started) &&
            stop_fault != MOTOR_FAULT_NONE)
            goto finished;
    }
    motor_states[motor].direction = PARK_AWAY_DIRECTION;
    home_seeking = 0U;
    if (!found)
    {
        stepper_request_stop(MOTOR_FAULT_HOME_NOT_FOUND);
        goto finished;
    }
    /* 固定退让，离开开关后建立零点；不能用固定步数直接假回零。 */
    for (steps = 0U; steps < STEPPER_HOME_BACKOFF_STEPS; steps++)
        if (!one_step(motor, PARK_AWAY_DIRECTION, 0U, STEPPER_HOME_DELAY_US, started))
            goto finished;
    if (!checked_delay(motor, PARK_AWAY_DIRECTION, 0U, 5000U, started))
        goto finished;
    if (limit_active(motor))
        stepper_request_stop(MOTOR_FAULT_HOME_STUCK);
    else
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (stop_fault == MOTOR_FAULT_NONE)
        {
            motor_states[motor].step_count = 0;
            homed[motor] = 1U;
        }
        __set_PRIMASK(mask);
    }
finished:
    finish_motion(motor);
}

MotorStatus stepper_get_status(MotorID motor)
{
    return valid_motor(motor) ? motor_states[motor].status : MOTOR_FAULT;
}

int32_t stepper_get_position(MotorID motor)
{
    return valid_motor(motor) ? motor_states[motor].step_count : 0;
}

void stepper_set_speed(MotorID motor, uint16_t delay_us)
{
    if (!valid_motor(motor) || active_motor != MOTOR_COUNT)
        return;
    if (delay_us < 800U) delay_us = 800U;
    if (delay_us > 5000U) delay_us = 5000U;
    motor_configs[motor].step_delay_us = delay_us;
}

void stepper_set_mode(MotorID motor, StepMode mode)
{
    if (!valid_motor(motor) || active_motor != MOTOR_COUNT ||
        (mode != STEP_FULL && mode != STEP_HALF))
        return;
    motor_configs[motor].mode = mode;
    homed[motor] = 0U;
}

void stepper_reset_position(MotorID motor)
{
    if (!valid_motor(motor) || active_motor != MOTOR_COUNT)
        return;
    motor_states[motor].step_count = 0;
    /* 人工修改计数不能替代物理回零。 */
    homed[motor] = 0U;
}
