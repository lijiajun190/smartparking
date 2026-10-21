/* Production driver, safety checks and FSM run against deterministic mock hardware. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_stepper.h"
#include "bsp_sensors.h"
#include "parking_fsm.h"
#include "motion_config.h"
#include "health_watch.h"

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expr); exit(1); } } while (0)
GPIO_TypeDef test_ports[3] = {{0}, {1}, {2}};
TestDebug test_debug;
uint32_t SystemCoreClock = 1000000U;
static TestDwt clock_regs;
static uint32_t irq_mask, micros;
static int physical[2], limit_mode[2], sticky[2], output[2], phase[2];
static unsigned pulses[2], all_pulses, trigger_at, injection;
static unsigned forbidden_output, tests;
static uint8_t emergency, short_pulse;
static uint8_t ir[2];
static const int patterns[8] = {1, 3, 2, 6, 4, 12, 8, 9};

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
uint32_t HAL_GetTick(void) { return micros / 1000U; }

static void inject(void)
{
    if (irq_mask || !trigger_at || all_pulses < trigger_at)
        return;
    trigger_at = 0U;
    if (injection == 1U)
    {
        emergency = 1U;
        stepper_sensor_irq();
        if (short_pulse) emergency = 0U;
    }
    else if (injection == 2U)
        stepper_request_stop(MOTOR_FAULT_STOP);
    else if (injection == 3U || injection == 4U)
    {
        limit_mode[injection - 3U] = 1;
        stepper_sensor_irq();
    }
    else if (injection == 5U)
        micros += STEPPER_COMMAND_TIMEOUT_MS * 1000U;
}

TestDwt *test_dwt(void)
{
    clock_regs.CYCCNT += 100U;
    micros += 100U;
    inject();
    return &clock_regs;
}
void vTaskDelay(unsigned ticks) { micros += ticks * 1000U; inject(); }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
    unsigned m = port->id;
    CHECK(m < 2U);
    if (value == GPIO_PIN_SET)
    {
        if (stepper_get_fault() != MOTOR_FAULT_NONE) forbidden_output++;
        output[m] |= pin;
    }
    else output[m] &= ~pin;
    /* The fourth write completes one phase or a coils-off operation. */
    if (pin == 8U && output[m] != 0)
    {
        int next = -1;
        int i;
        for (i = 0; i < 8; i++)
            if (patterns[i] == output[m]) next = i;
        CHECK(next >= 0);
        int delta = (next - phase[m] + 8) % 8;
        CHECK(delta == 1 || delta == 7);
        physical[m] += delta == 1 ? 1 : -1;
        phase[m] = next;
        pulses[m]++;
        all_pulses++;
    }
}

uint8_t sensor_read(SensorId id)
{
    if (id == SENSOR_IR_1 || id == SENSOR_IR_2) return ir[id];
    if (id == SENSOR_EMERGENCY) return emergency;
    if (id == SENSOR_LIMIT_1 || id == SENSOR_LIMIT_2)
    {
        unsigned m = id == SENSOR_LIMIT_1 ? 0U : 1U;
        if (limit_mode[m] == -1) return 0U;
        if (limit_mode[m] == 1) return 1U;
        if (limit_mode[m] == 2 && physical[m] <= 0) sticky[m] = 1;
        return sticky[m] || physical[m] <= 0;
    }
    return 0U;
}
uint8_t sensor_emergency_active(void) { return emergency; }

static void reset_hardware(int a, int b)
{
    memset(limit_mode, 0, sizeof(limit_mode));
    memset(sticky, 0, sizeof(sticky));
    memset(output, 0, sizeof(output));
    memset(phase, 0, sizeof(phase));
    memset(pulses, 0, sizeof(pulses));
    physical[0] = a; physical[1] = b;
    emergency = short_pulse = 0U;
    memset(ir, 0, sizeof(ir));
    trigger_at = injection = forbidden_output = all_pulses = 0U;
    micros = irq_mask = 0U;
    stepper_init();
}
static void expect_fault(MotorFault fault)
{
    CHECK(stepper_get_fault() == fault);
    CHECK(output[0] == 0 && output[1] == 0);
    CHECK(!stepper_is_homed(MOTOR_A) && !stepper_is_homed(MOTOR_B));
    CHECK(forbidden_output == 0U);
}
static void arm(unsigned kind, unsigned after)
{
    injection = kind;
    trigger_at = all_pulses + after;
}
static void pass(const char *name) { tests++; printf("PASS %s\n", name); }

static void driver_tests(void)
{
    reset_hardware(120, 80);
    stepper_home(MOTOR_A);
    CHECK(stepper_get_fault() == MOTOR_FAULT_NONE && stepper_is_homed(MOTOR_A));
    CHECK(physical[0] == 32 && pulses[0] == 152U && pulses[1] == 0U);
    CHECK(stepper_get_position(MOTOR_A) == 0);
    stepper_move_steps(MOTOR_A, 100U, DIR_CW);
    stepper_move_steps(MOTOR_A, 50U, DIR_CCW);
    CHECK(stepper_get_position(MOTOR_A) == 50 && physical[0] == 82);
    stepper_home(MOTOR_A);
    CHECK(stepper_is_homed(MOTOR_A) && physical[0] == 32);
    unsigned before = all_pulses;
    stepper_move_steps(MOTOR_A, 0U, DIR_CW);
    CHECK(all_pulses == before && stepper_get_status(MOTOR_A) == MOTOR_IDLE);
    pass("seek, stop, backoff, zero; repeat home; zero-step move");

    reset_hardware(-3, 10);
    stepper_home(MOTOR_A);
    CHECK(stepper_is_homed(MOTOR_A) && physical[0] == 32);
    pass("initially pressed limit releases and is approached again");

    reset_hardware(100, 10); limit_mode[0] = -1;
    stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_HOME_NOT_FOUND);
    CHECK(pulses[0] == STEPPER_HOME_SEEK_STEPS);
    CHECK(stepper_get_position(MOTOR_A) != 0);
    pass("missing limit bounded by seek step budget, no false zero");

    reset_hardware(0, 10); limit_mode[0] = 1;
    stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_HOME_STUCK);
    CHECK(pulses[0] == STEPPER_HOME_RELEASE_STEPS);
    pass("initially stuck limit bounded by release budget");

    reset_hardware(5, 10); limit_mode[0] = 2;
    stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_HOME_STUCK);
    CHECK(pulses[0] == 5U + STEPPER_HOME_BACKOFF_STEPS);
    pass("limit fails to release after backoff, no false zero");

    unsigned m;
    for (m = 0U; m < 2U; m++)
    {
        reset_hardware(40, 60);
        stepper_home(MOTOR_A); stepper_home(MOTOR_B);
        arm(1U, 5U); short_pulse = 1U;
        stepper_move_steps((MotorID)m, 100U, DIR_CW);
        expect_fault(MOTOR_FAULT_EMERGENCY);
        CHECK(stepper_get_position((MotorID)m) == 5);
        before = all_pulses;
        stepper_move_steps((MotorID)m, 100U, DIR_CW);
        CHECK(all_pulses == before);
        emergency = 1U; CHECK(!stepper_clear_fault());
        emergency = 0U; CHECK(stepper_clear_fault());
        CHECK(!stepper_is_homed((MotorID)m));
        stepper_move_steps((MotorID)m, 1U, DIR_CW);
        expect_fault(MOTOR_FAULT_NOT_HOMED);
    }
    pass("short emergency pulse on either axis stops both; release cannot resume");

    reset_hardware(40, 60); stepper_home(MOTOR_A);
    arm(2U, 3U); stepper_move_steps(MOTOR_A, 100U, DIR_CW);
    expect_fault(MOTOR_FAULT_STOP);
    CHECK(stepper_get_status(MOTOR_A) == MOTOR_STOPPED);
    CHECK(stepper_get_position(MOTOR_A) == 3);
    pass("asynchronous STOP aborts loop and is not IDLE");

    for (m = 0U; m < 2U; m++)
    {
        reset_hardware(40, 60); stepper_home((MotorID)m);
        stepper_move_steps((MotorID)m, 100U, DIR_CW);
        arm(3U + m, 3U); stepper_move_steps((MotorID)m, 100U, DIR_CCW);
        expect_fault(MOTOR_FAULT_LIMIT);
        CHECK(stepper_get_position((MotorID)m) == 97);
    }
    pass("unexpected limit toward home aborts either axis mid-move");

    reset_hardware(10, 10); stepper_home(MOTOR_A);
    limit_mode[0] = 1; stepper_move_steps(MOTOR_A, 5U, DIR_CW);
    CHECK(stepper_get_fault() == MOTOR_FAULT_NONE && stepper_get_position(MOTOR_A) == 5);
    pass("moving away from active home limit remains possible");

    reset_hardware(120, 10); arm(1U, 10U); stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_EMERGENCY);
    CHECK(stepper_get_position(MOTOR_A) == -10);
    reset_hardware(4, 10); arm(1U, 7U); stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_EMERGENCY);
    CHECK(physical[0] == 3 && stepper_get_position(MOTOR_A) != 0);
    pass("emergency during seek and backoff cannot establish zero");

    reset_hardware(120, 10); arm(5U, 2U); stepper_home(MOTOR_A);
    expect_fault(MOTOR_FAULT_TIMEOUT);
    CHECK(pulses[0] == 2U);
    pass("command timeout aborts before further phases");

    reset_hardware(10, 10); stepper_move_steps(MOTOR_A, 10U, DIR_CW);
    expect_fault(MOTOR_FAULT_NOT_HOMED); CHECK(all_pulses == 0U);
    reset_hardware(10, 10); stepper_home(MOTOR_A);
    stepper_move_steps(MOTOR_A, 1U, DIR_CCW);
    expect_fault(MOTOR_FAULT_TRAVEL);
    reset_hardware(10, 10); stepper_move_steps((MotorID)99, 1U, DIR_CW);
    stepper_service_stop(); expect_fault(MOTOR_FAULT_COMMAND);
    pass("unhomed moves, software travel violation and invalid axis rejected");

    reset_hardware(20, 30); stepper_home(MOTOR_A); stepper_home(MOTOR_B);
    stepper_request_stop(MOTOR_FAULT_EMERGENCY); stepper_service_stop();
    expect_fault(MOTOR_FAULT_EMERGENCY); CHECK(stepper_clear_fault());
    stepper_home(MOTOR_B); stepper_home(MOTOR_A);
    CHECK(stepper_is_homed(MOTOR_A) && stepper_is_homed(MOTOR_B));
    pass("manual clear followed by B/A homing restores motion eligibility");
}

static ParkingMotionType requested_type;
static MotorID requested_motor;
static uint8_t accept_request = 1U;
static unsigned requests;
static unsigned persist_calls;
static uint8_t persist_ok = 1U, saved_pending;
static uint8_t persist_mock(const ParkingContext *ctx)
{
    persist_calls++;
    saved_pending = ctx->pending;
    return persist_ok;
}
static uint8_t request(ParkingMotionType type, MotorID motor,
                       uint32_t steps, MotorDirection direction, uint32_t command_id)
{
    (void)steps; (void)direction; CHECK(command_id != 0U);
    requests++; requested_type = type; requested_motor = motor;
    return accept_request;
}
static void event(ParkingContext *ctx, AppEventType type, unsigned value)
{
    AppEvent e = {0}; e.type = type; e.value = value; e.tick = 1234U;
    e.command_id = ctx->expected_command_id;
    e.command_type = ctx->expected_command_type;
    e.uid[0] = 42U;
    parking_handle_event(ctx, &e, request);
}
static void fsm_tests(void)
{
    ParkingContext ctx;
    reset_hardware(20, 20);
    parking_init(&ctx); parking_begin_home(&ctx, request, 0U);
    CHECK(ctx.state == PARK_MOVING_HOME && requested_motor == MOTOR_B);
    CHECK(requested_type == PARK_MOTION_HOME);
    unsigned before = requests;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(requests == before && ctx.state == PARK_MOVING_HOME);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(requested_motor == MOTOR_A && ctx.state == PARK_MOVING_HOME);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A); CHECK(ctx.state == PARK_IDLE);
    pass("startup waits for B then A HOME; wrong-axis completion ignored");

    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    CHECK(ctx.state == PARK_SELECT_ACTION);
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    CHECK(ctx.state == PARK_MOVING_IN && requested_type == PARK_MOTION_MOVE);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(requested_type == PARK_MOTION_MOVE && requested_motor == MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(requested_type == PARK_MOTION_HOME && requested_motor == MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(requested_type == PARK_MOTION_HOME && requested_motor == MOTOR_A);
    CHECK(!ctx.slots[0].occupied);
    ir[0] = 1U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(ctx.state == PARK_IDLE && ctx.slots[0].occupied);
    CHECK(ctx.slots[0].start_tick == 1234U);
    pass("parking return uses real B/A HOME and commits only after both complete");

    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    CHECK(ctx.state == PARK_WAIT_PAYMENT);
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(requested_type == PARK_MOTION_HOME && requested_motor == MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(requested_type == PARK_MOTION_HOME && requested_motor == MOTOR_A);
    ir[0] = 0U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(ctx.state == PARK_IDLE && !ctx.slots[0].occupied);
    pass("retrieval return also uses real B/A HOME");

    unsigned state;
    for (state = PARK_IDLE; state <= PARK_FAULT; state++)
    {
        ctx.state = (ParkingState)state;
        event(&ctx, APP_EVENT_EMERGENCY_ON, 1U);
        CHECK(ctx.state == PARK_FAULT && ctx.motion_phase == MOTION_PHASE_IDLE);
        event(&ctx, APP_EVENT_EMERGENCY_OFF, 0U); CHECK(ctx.state == PARK_FAULT);
    }
    accept_request = 0U; event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_FAULT);
    accept_request = 1U; event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_MOVING_HOME && requested_type == PARK_MOTION_RECOVER_HOME);
    event(&ctx, APP_EVENT_MOTION_FAULT, MOTOR_B); CHECK(ctx.state == PARK_FAULT);
    pass("all states latch faults; rejected recovery and homing faults remain FAULT");

    parking_init(&ctx); ctx.state = PARK_IDLE;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    accept_request = 0U; event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    CHECK(ctx.state == PARK_FAULT && !ctx.slots[0].occupied);
    pass("failed command submission cannot advance or commit occupancy");
}

static void boundary_tests(void)
{
    ParkingContext ctx;
    ParkingSlot checked[PARKING_SLOT_COUNT] = {{0}};
    AppEvent stale = {0};
    unsigned i, before;
    accept_request = 1U;
    reset_hardware(20, 20);
    CHECK(motion_config_valid());
    CHECK(PARKING_SLOTS[2].occupancy_sensor == SENSOR_ID_COUNT);
    CHECK(PARKING_SLOTS[3].occupancy_sensor == SENSOR_ID_COUNT);
    parking_init(&ctx); ctx.state = PARK_IDLE;
    for (i = 0U; i < PARKING_SLOT_COUNT; i++) {
        ctx.slots[i].occupied = 1U; ctx.slots[i].uid[0] = (uint8_t)(i + 1U);
    }
    before = requests;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    CHECK(ctx.state == PARK_FULL && requests == before);
    CHECK(stepper_get_fault() == MOTOR_FAULT_NONE);
    ctx.slots[2].uid[0] = 42U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    CHECK(ctx.state == PARK_WAIT_PAYMENT && ctx.selected_slot == 2U);
    pass("full is normal; existing UID can still retrieve; slot configuration validates");

    parking_init(&ctx); ctx.state = PARK_IDLE;
    ctx.persist = persist_mock; persist_ok = 1U;
    persist_calls = 0U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    CHECK(ctx.pending == 1U && persist_calls == 1U && saved_pending == 1U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    stale.type = APP_EVENT_MOTION_DONE; stale.value = MOTOR_B;
    stale.command_id = ctx.expected_command_id;
    stale.command_type = ctx.expected_command_type;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B); /* 同轴MOVE完成，接下来是HOME */
    before = requests;
    parking_handle_event(&ctx, &stale, request);
    CHECK(requests == before && ctx.motion_phase == MOTION_PHASE_RETURN_V);
    stale.command_id = ctx.expected_command_id; /* 新序号但种类错误 */
    parking_handle_event(&ctx, &stale, request);
    CHECK(requests == before && ctx.motion_phase == MOTION_PHASE_RETURN_V);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    ir[0] = 1U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(ctx.state == PARK_IDLE && ctx.slots[0].occupied && !ctx.pending);
    CHECK(persist_calls == 2U && saved_pending == 0U);
    pass("pending saved before motion; stale same-axis and wrong-type completion ignored; final commit");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_IDLE; ctx.persist = persist_mock;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    persist_ok = 0U; before = requests;
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    CHECK(ctx.state == PARK_FAULT && ctx.storage_fault && ctx.reconcile_required);
    CHECK(requests == before && !ctx.slots[0].occupied);
    persist_ok = 1U;
    pass("failed initial journal write prevents any movement");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_IDLE; ctx.persist = persist_mock;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    event(&ctx, APP_EVENT_EMERGENCY_ON, 1U);
    before = requests;
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_FAULT && ctx.pending && requests == before);
    emergency = 1U; CHECK(!parking_reconcile(&ctx, checked));
    emergency = 0U; ir[0] = 1U; CHECK(!parking_reconcile(&ctx, checked));
    ir[0] = 0U;
    checked[2].occupied = checked[3].occupied = 1U;
    checked[2].uid[0] = checked[3].uid[0] = 10U;
    CHECK(!parking_reconcile(&ctx, checked));
    checked[3].uid[0] = 11U;
    CHECK(parking_reconcile(&ctx, checked));
    CHECK(!ctx.reconcile_required && !ctx.pending && ctx.state == PARK_FAULT);
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_MOVING_HOME && requested_type == PARK_MOTION_RECOVER_HOME);
    pass("interrupted transfer requires reconciliation; reject emergency, IR mismatch and duplicate UID");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_FAULT;
    emergency = 1U; before = requests;
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_FAULT && requests == before);
    emergency = 0U; ctx.next_command_id = 0xFFFFFFFFU;
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.expected_command_id == 1U);
    pass("held emergency blocks recovery; command ID skips zero on wrap");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_IDLE; ctx.persist = persist_mock;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    ir[0] = 1U; before = requests;
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    CHECK(ctx.state == PARK_IDLE && requests == before && !ctx.pending);
    ir[0] = 0U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(ctx.state == PARK_FAULT && ctx.pending && !ctx.slots[0].occupied);
    pass("recheck selected slot; missing IR transfer confirmation cannot commit occupancy");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_IDLE; ctx.persist = persist_mock;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    ir[0] = 1U; persist_ok = 0U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(ctx.state == PARK_FAULT && ctx.storage_fault && ctx.reconcile_required);
    before = requests; event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(requests == before && ctx.state == PARK_FAULT);
    persist_ok = 1U;
    pass("failed final commit locks service until records are reconciled");

    reset_hardware(3000, 20);
    health_init(HAL_GetTick()); stepper_home(MOTOR_A);
    CHECK(HAL_GetTick() > HEALTH_TIMEOUT_MS && stepper_is_homed(MOTOR_A));
    (void)health_check(HAL_GetTick());
    CHECK((health_failed_mask() & (1U << HEALTH_MOTION)) == 0U);
    pass("long valid HOME reports per-step motion progress to watchdog supervisor");
}

static uint32_t fake_epoch;
static unsigned successful_notices, fault_notices;
static uint32_t clock_mock(void) { return fake_epoch; }
static void notice_mock(const ParkingContext *ctx, uint8_t kind)
{
    CHECK(ctx != NULL);
    if (kind == 3U) fault_notices++;
    else { CHECK(kind == 1U || kind == 2U); successful_notices++; }
}
static void fee_tests(void)
{
    ParkingContext ctx;
    AppEvent card = {0};
    unsigned before;
    reset_hardware(20, 20); accept_request = persist_ok = 1U;
    parking_init(&ctx); ctx.state = PARK_IDLE;
    ctx.clock = clock_mock; ctx.slots[2].occupied = 1U; ctx.slots[2].uid[0] = 42U;
    ctx.slots[2].start_epoch = 1700000000U; ctx.slots[2].time_uncertain = 1U;
    fake_epoch = 1700001801U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    CHECK(ctx.state == PARK_WAIT_PAYMENT && !ctx.quote_uncertain);
    CHECK(ctx.quote_seconds == 1801U && ctx.quote_cents == 1000U);
    CHECK(!parking_set_fee(&ctx, 0U));
    pass("restored RTC start time produces full cross-reboot fee and prevents overriding a valid quote");

    ctx.state = PARK_IDLE; fake_epoch = 0U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U);
    before = requests;
    CHECK(ctx.quote_uncertain);
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_WAIT_PAYMENT && requests == before);
    CHECK(parking_set_fee(&ctx, 1250U) && ctx.quote_cents == 1250U && !ctx.quote_uncertain);
    event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    CHECK(ctx.state == PARK_MOVING_OUT && requests == before + 1U);
    CHECK(!parking_set_fee(&ctx, 0U));
    ctx.state = PARK_IDLE; fake_epoch = 1699999999U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U); CHECK(ctx.quote_uncertain);
    pass("invalid or backward RTC blocks payment until manual fee assessment; assessment cannot bypass business state");

    parking_init(&ctx); ctx.state = PARK_IDLE;
    ctx.slots[2].occupied = 1U; ctx.slots[2].uid[0] = 42U;
    ctx.slots[2].start_tick = 0xFFFFFF00U;
    parking_poll(&ctx, 1744U);
    card.type = APP_EVENT_CARD_DETECTED; card.uid[0] = 42U; card.tick = 1740U;
    parking_handle_event(&ctx, &card, request);
    CHECK(ctx.quote_seconds == 1U && ctx.quote_cents == 500U && !ctx.quote_uncertain);
    /* 用小于半个tick周期的多次poll模拟持续运行超过一个完整49.7天周期。 */
    parking_poll(&ctx, 0x70000000U); parking_poll(&ctx, 0xE0000000U);
    parking_poll(&ctx, 0x50000000U); parking_poll(&ctx, 0xC0000000U);
    ctx.state = PARK_IDLE; card.tick = 0xC0000000U;
    parking_handle_event(&ctx, &card, request);
    CHECK(ctx.quote_seconds > 4294967U && !ctx.quote_uncertain);
    pass("periodic accounting survives multiple tick wraps and queued card timestamps earlier than the last poll");

    ctx.state = PARK_SELECT_ACTION; ctx.selection_tick = 0xFFFFFF00U;
    parking_poll(&ctx, 0xFFFFFF00U + 29999U); CHECK(ctx.state == PARK_SELECT_ACTION);
    parking_poll(&ctx, 0xFFFFFF00U + 30000U); CHECK(ctx.state == PARK_IDLE);
    ctx.state = PARK_WAIT_PAYMENT; ctx.selection_tick = 100U; ctx.quote_uncertain = 1U;
    parking_poll(&ctx, 30100U); CHECK(ctx.state == PARK_IDLE && !ctx.quote_uncertain);
    pass("abandoned selection and payment expire after 30 seconds including across tick wrap");

    reset_hardware(20, 20);
    parking_init(&ctx); ctx.state = PARK_IDLE; ctx.persist = persist_mock;
    ctx.clock = clock_mock; ctx.notice = notice_mock; fake_epoch = 1700000000U;
    successful_notices = fault_notices = 0U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U); event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A); event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    CHECK(!successful_notices); ir[0] = 1U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(successful_notices == 1U && ctx.slots[0].start_epoch == fake_epoch && !ctx.slots[0].time_uncertain);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A); CHECK(successful_notices == 1U);
    fake_epoch += 1801U;
    event(&ctx, APP_EVENT_CARD_DETECTED, 0U); event(&ctx, APP_EVENT_KEY_LONG_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A); event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B); ir[0] = 0U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(successful_notices == 2U && !ctx.slots[0].occupied && ctx.quote_cents == 1000U);
    pass("successful journal commit emits one parking/retrieval notice with RTC start and final fee");

    event(&ctx, APP_EVENT_CARD_DETECTED, 0U); event(&ctx, APP_EVENT_KEY_PRESS, 0U);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A); event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B);
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_B); ir[0] = 1U; persist_ok = 0U;
    event(&ctx, APP_EVENT_MOTION_DONE, MOTOR_A);
    CHECK(successful_notices == 2U && fault_notices == 1U && ctx.state == PARK_FAULT);
    event(&ctx, APP_EVENT_MOTION_FAULT, 0U); CHECK(fault_notices == 1U);
    persist_ok = 1U;
    pass("failed final persistence emits no false success; repeated latched fault emits one alert");
}

int main(void)
{
    driver_tests(); fsm_tests(); boundary_tests(); fee_tests();
    printf("%u test groups passed\n", tests);
    return 0;
}
