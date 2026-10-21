#include "parking_fsm.h"
#include "motion_config.h"
#include "bsp_sensors.h"
#include "billing.h"
#include "app_config.h"
#include <string.h>


static int parking_uid_equal(const uint8_t *left,
                             const uint8_t *right)
{
    return (memcmp(left, right, 4U) == 0) ? 1 : 0;
}

/**
 * @brief 智能车位选择：优先级 + 传感器验证
 * @param context 停车上下文
 * @return 推荐的车位编号；-1 表示无空位
 */
static int parking_find_free_slot(const ParkingContext *context)
{
    uint8_t i, slot;
    uint8_t sensor_occupied;

    /* 按优先级顺序查找空车位 */
    for (i = 0U; i < PARKING_SLOT_COUNT; i++)
    {
        for (slot = 0U; slot < PARKING_SLOT_COUNT; slot++)
            if (PARKING_SLOTS[slot].priority == i) break;
        if (slot == PARKING_SLOT_COUNT) return -1;

        /* 软件状态检查 */
        if (context->slots[slot].occupied != 0U)
        {
            continue;  /* 软件记录已占用，跳过 */
        }

        /* 传感器验证（如果该车位有传感器） */
        if (PARKING_SLOTS[slot].occupancy_sensor < SENSOR_ID_COUNT)
        {
            sensor_occupied = sensor_read(PARKING_SLOTS[slot].occupancy_sensor);

            if (sensor_occupied != 0U)
            {
                /* 传感器检测到有车，但软件状态未记录 → 异常情况 */
                /* 跳过该车位，避免冲突 */
                continue;
            }
        }

        /* 找到空闲车位 */
        return (int)slot;
    }

    return -1;  /* 无空位 */
}

static int parking_find_uid_slot(const ParkingContext *context,
                                 const uint8_t *uid)
{
    uint8_t i;

    for (i = 0U; i < PARKING_SLOT_COUNT; i++)
    {
        if ((context->slots[i].occupied != 0U) &&
            parking_uid_equal(context->slots[i].uid, uid))
        {
            return (int)i;
        }
    }

    return -1;
}

void parking_init(ParkingContext *context)
{
    if (context == NULL)
    {
        return;
    }

    memset(context, 0, sizeof(ParkingContext));
    context->state = PARK_MOVING_HOME;
    context->motion_phase = MOTION_PHASE_IDLE;
}

static void parking_fault(ParkingContext *context)
{
    uint8_t notify = context->state != PARK_FAULT;
    if (context->pending) context->reconcile_required = 1U;
    context->expected_command_id = 0U;
    context->motion_phase = MOTION_PHASE_IDLE;
    context->state = PARK_FAULT;
    if (notify && context->notice) context->notice(context, 3U);
}

static void parking_quote(ParkingContext *context, uint32_t tick)
{
    const ParkingSlot *slot = &context->slots[context->selected_slot];
    const BillingConfig rate = {APP_BILLING_UNIT_SECONDS, APP_BILLING_RATE_CENTS};
    uint32_t epoch = context->clock ? context->clock() : 0U;
    context->quote_uncertain = slot->time_uncertain;
    if (epoch && slot->start_epoch && epoch >= slot->start_epoch) {
        context->quote_seconds = epoch - slot->start_epoch;
        context->quote_uncertain = 0U;
    } else {
        uint64_t elapsed = slot->elapsed_ms;
        uint32_t delta = tick - slot->start_tick;
        uint64_t seconds;
        /* 队列中的刷卡时间可能略早于最近一次poll；不能把负差当成49天。 */
        if ((int32_t)delta >= 0) elapsed += delta;
        else {
            uint32_t lag = slot->start_tick - tick;
            elapsed = elapsed >= lag ? elapsed - lag : 0U;
        }
        seconds = elapsed / 1000U;
        context->quote_seconds = seconds > UINT32_MAX ? UINT32_MAX : (uint32_t)seconds;
        if (slot->start_epoch) context->quote_uncertain = 1U; /* RTC失效/回拨不能退回uptime误计费 */
    }
    context->quote_cents = billing_calculate_cents(context->quote_seconds, &rate);
}

void parking_poll(ParkingContext *context, uint32_t now)
{
    unsigned i;
    if (!context) return;
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        if (context->slots[i].occupied) {
            context->slots[i].elapsed_ms += (uint32_t)(now - context->slots[i].start_tick);
            context->slots[i].start_tick = now;
        }
    }
    if (context && (context->state == PARK_SELECT_ACTION || context->state == PARK_WAIT_PAYMENT) &&
        (uint32_t)(now - context->selection_tick) >= APP_SELECTION_TIMEOUT_MS) {
        context->state = PARK_IDLE;
        context->quote_uncertain = 0U;
    }
}
uint8_t parking_set_fee(ParkingContext *context, uint32_t cents)
{
    if (!context || context->state != PARK_WAIT_PAYMENT || !context->quote_uncertain) return 0U;
    context->quote_cents = cents;
    context->quote_uncertain = 0U;
    return 1U;
}

static void parking_submit(ParkingContext *context, ParkingMoveCallback callback,
                           ParkingMotionType type, MotorID motor, uint32_t steps)
{
    if (++context->next_command_id == 0U) ++context->next_command_id;
    context->expected_command_id = context->next_command_id;
    context->expected_motor = motor;
    context->expected_command_type = (uint32_t)type;
    if (callback == NULL || !callback(type, motor, steps, PARK_AWAY_DIRECTION,
                                     context->expected_command_id))
        parking_fault(context);
}

static uint8_t parking_persist(ParkingContext *context)
{
    if (context->persist && !context->persist(context)) {
        context->storage_fault = 1U;
        context->reconcile_required = 1U;
        stepper_request_stop(MOTOR_FAULT_COMMAND);
        parking_fault(context);
        return 0U;
    }
    return 1U;
}

uint8_t parking_reconcile(ParkingContext *context, const ParkingSlot *slots)
{
    uint8_t i, j;
    if (!context || !slots || context->state != PARK_FAULT ||
        sensor_emergency_active() || stepper_get_status(MOTOR_A) == MOTOR_RUNNING ||
        stepper_get_status(MOTOR_B) == MOTOR_RUNNING)
        return 0U;
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        SensorId sensor = PARKING_SLOTS[i].occupancy_sensor;
        if (slots[i].occupied > 1U ||
            (sensor < SENSOR_ID_COUNT && sensor_read(sensor) != slots[i].occupied))
            return 0U;
        for (j = 0U; j < i; j++)
            if (slots[i].occupied && slots[j].occupied &&
                parking_uid_equal(slots[i].uid, slots[j].uid)) return 0U;
    }
    memcpy(context->slots, slots, sizeof(context->slots));
    context->pending = 0U;
    if (!parking_persist(context)) return 0U;
    context->storage_fault = context->reconcile_required = 0U;
    return 1U;
}

void parking_begin_home(ParkingContext *context, ParkingMoveCallback callback,
                        uint8_t recover)
{
    if (context == NULL)
        return;
    if (context->reconcile_required || context->storage_fault ||
        sensor_emergency_active() || !motion_config_valid()) {
        parking_fault(context);
        return;
    }
    context->state = PARK_MOVING_HOME;
    context->motion_phase = MOTION_PHASE_RETURN_V;
    parking_submit(context, callback,
                   recover ? PARK_MOTION_RECOVER_HOME : PARK_MOTION_HOME,
                   MOTOR_B, 0U);
}

void parking_handle_event(ParkingContext *context,
                          const AppEvent *event,
                          ParkingMoveCallback move_callback)
{
    int slot;
    if (context == NULL || event == NULL)
        return;

    /* 所有状态都响应故障，释放急停事件不会恢复业务。 */
    if (event->type == APP_EVENT_EMERGENCY_ON ||
        event->type == APP_EVENT_MOTION_FAULT ||
        event->type == APP_EVENT_MOTION_TIMEOUT)
    {
        stepper_request_stop(event->type == APP_EVENT_EMERGENCY_ON ?
                             MOTOR_FAULT_EMERGENCY : MOTOR_FAULT_COMMAND);
        parking_fault(context);
        return;
    }

    /* 电机、命令种类和唯一序号必须全部匹配；拒绝迟到/重复完成事件。 */
    if (event->type == APP_EVENT_MOTION_DONE)
    {
        if (context->motion_phase == MOTION_PHASE_IDLE ||
            event->value != (uint32_t)context->expected_motor ||
            context->expected_command_id == 0U ||
            event->command_id != context->expected_command_id ||
            event->command_type != context->expected_command_type)
            return;
        context->expected_command_id = 0U;
    }

    switch (context->state)
    {
        case PARK_IDLE:
        case PARK_FULL:
            if (event->type == APP_EVENT_CARD_DETECTED)
            {
                memcpy(context->current_uid, event->uid, 4U);
                context->selection_tick = event->tick;
                slot = parking_find_uid_slot(context, event->uid);
                if (slot >= 0)
                {
                    context->selected_slot = (uint8_t)slot;
                    context->state = PARK_WAIT_PAYMENT;
                    parking_quote(context, event->tick);
                }
                else
                {
                    slot = parking_find_free_slot(context);
                    if (slot < 0)
                    {
                        context->state = PARK_FULL;
                        break;
                    }
                    context->selected_slot = (uint8_t)slot;
                    context->state = PARK_SELECT_ACTION;
                }
            }
            break;

        case PARK_SELECT_ACTION:
        case PARK_WAIT_PAYMENT:
            if ((context->state == PARK_SELECT_ACTION &&
                 event->type == APP_EVENT_KEY_PRESS) ||
                (context->state == PARK_WAIT_PAYMENT &&
                 event->type == APP_EVENT_KEY_LONG_PRESS))
            {
                ParkingPosition pos = motion_get_slot_position(context->selected_slot);
                /* 时间不可靠必须先由维护人员核定费用，长按不能绕过。 */
                if (context->state == PARK_WAIT_PAYMENT && context->quote_uncertain) break;
                /* 存车确认时重新检查，避免刷卡后车位被外部车辆占用。 */
                SensorId sensor = PARKING_SLOTS[context->selected_slot].occupancy_sensor;
                if (context->state == PARK_SELECT_ACTION &&
                    (context->slots[context->selected_slot].occupied ||
                     (sensor < SENSOR_ID_COUNT && sensor_read(sensor)))) {
                    context->state = PARK_IDLE;
                    break;
                }
                context->pending = context->state == PARK_SELECT_ACTION ? 1U : 2U;
                /* 必须先落盘未完成事务，成功才允许第一条运动命令入队。 */
                if (!parking_persist(context)) break;
                context->state = context->state == PARK_SELECT_ACTION ?
                                  PARK_MOVING_IN : PARK_MOVING_OUT;
                context->motion_phase = MOTION_PHASE_HORIZONTAL;
                parking_submit(context, move_callback, PARK_MOTION_MOVE,
                               MOTOR_A, pos.motor_a_steps);
            }
            break;

        case PARK_MOVING_HOME:
            if (event->type == APP_EVENT_MOTION_DONE)
            {
                if (context->motion_phase == MOTION_PHASE_RETURN_V)
                {
                    context->motion_phase = MOTION_PHASE_RETURN_H;
                    parking_submit(context, move_callback, PARK_MOTION_HOME, MOTOR_A, 0U);
                }
                else if (context->motion_phase == MOTION_PHASE_RETURN_H)
                {
                    context->motion_phase = MOTION_PHASE_IDLE;
                    context->state = PARK_IDLE;
                }
            }
            break;

        case PARK_MOVING_IN:
        case PARK_MOVING_OUT:
            if (event->type == APP_EVENT_MOTION_DONE)
            {
                ParkingPosition pos = motion_get_slot_position(context->selected_slot);
                switch (context->motion_phase)
                {
                    case MOTION_PHASE_HORIZONTAL:
                        context->motion_phase = MOTION_PHASE_VERTICAL;
                        parking_submit(context, move_callback, PARK_MOTION_MOVE,
                                       MOTOR_B, pos.motor_b_steps);
                        break;
                    case MOTION_PHASE_VERTICAL:
                        context->motion_phase = MOTION_PHASE_RETURN_V;
                        parking_submit(context, move_callback, PARK_MOTION_HOME, MOTOR_B, 0U);
                        break;
                    case MOTION_PHASE_RETURN_V:
                        context->motion_phase = MOTION_PHASE_RETURN_H;
                        parking_submit(context, move_callback, PARK_MOTION_HOME, MOTOR_A, 0U);
                        break;
                    case MOTION_PHASE_RETURN_H:
                        slot = context->selected_slot;
                        /* 有独立IR的车位必须与交接结果相符；无反馈车位依赖人工标定。 */
                        if (PARKING_SLOTS[slot].occupancy_sensor < SENSOR_ID_COUNT &&
                            sensor_read(PARKING_SLOTS[slot].occupancy_sensor) !=
                                (context->state == PARK_MOVING_IN ? 1U : 0U)) {
                            stepper_request_stop(MOTOR_FAULT_COMMAND);
                            parking_fault(context);
                            break;
                        }
                        if (context->state == PARK_MOVING_IN)
                        {
                            context->slots[slot].occupied = 1U;
                            memcpy(context->slots[slot].uid, context->current_uid, 4U);
                            context->slots[slot].start_tick = event->tick;
                            context->slots[slot].time_uncertain = 0U;
                            context->slots[slot].start_epoch = context->clock ? context->clock() : 0U;
                        }
                        else
                        {
                            memset(&context->slots[slot], 0, sizeof(ParkingSlot));
                        }
                        context->pending = 0U;
                        if (!parking_persist(context)) break;
                        if (context->notice) context->notice(context,
                            context->state == PARK_MOVING_IN ? 1U : 2U);
                        context->motion_phase = MOTION_PHASE_IDLE;
                        context->state = PARK_IDLE;
                        break;
                    default:
                        parking_fault(context);
                        break;
                }
            }
            break;

        case PARK_FAULT:
            if (event->type == APP_EVENT_KEY_LONG_PRESS)
                parking_begin_home(context, move_callback, 1U);
            break;

        default:
            parking_fault(context);
            break;
    }
}

ParkingState parking_get_state(const ParkingContext *context)
{
    if (context == NULL)
    {
        return PARK_FAULT;
    }

    return context->state;
}
