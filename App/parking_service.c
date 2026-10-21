#include "parking_service.h"
#include "console_command.h"
#include "parking_store.h"
#include "app_config.h"
#include "bsp_console.h"
#include "bsp_clock.h"
#include "bsp_sensors.h"
#include "bsp_sim800.h"
#include "motion_config.h"
#include "health_watch.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

typedef struct { char phone[20], text[161]; } SmsMessage;
static osMessageQueueId_t sms_queue;
static ParkingIdleCallback is_idle;
static ParkingDiagnosticsCallback print_diagnostics;
static osThreadId_t gsm_thread;
static char sms_phone[20] = APP_SMS_PHONE;
static volatile uint8_t sms_status;
static volatile uint32_t sms_drops, sms_failures;
static ParkingSlot staged[PARKING_SLOT_COUNT];
static uint8_t staged_mask, staging;
static const osThreadAttr_t gsm_attributes = {
    .name = "GsmTask", .stack_size = 4096U, .priority = osPriorityBelowNormal
};

/* 每条通知最多尝试3次；长AT等待仅阻塞此任务，驱动等待循环持续报告进度。
 * 发送超时可能已被运营商接收，重试可能重复，正文含UID便于核对。 */
static void gsm_task(void *argument)
{
    SmsMessage message;
    uint8_t connected = 0U;
    unsigned attempt;
    (void)argument;
    for (;;) {
        health_beat(HEALTH_GSM, HAL_GetTick());
        if (osMessageQueueGet(sms_queue, &message, NULL, 100U) != osOK) continue;
        for (attempt = 0U; attempt < APP_SMS_MAX_ATTEMPTS; attempt++) {
            health_beat(HEALTH_GSM, HAL_GetTick());
            sms_status = 1U;
            if (!connected) connected = gsm_init() == GSM_OK;
            if (connected && gsm_send_sms(message.phone, message.text) == GSM_OK) {
                sms_status = 2U;
                printf("[SMS] accepted by modem/network\r\n");
                break;
            }
            connected = 0U;
            sms_status = 3U;
            osDelay(500U);
        }
        if (attempt == APP_SMS_MAX_ATTEMPTS) {
            sms_failures++;
            printf("[SMS] failed after %u attempts\r\n", attempt);
        }
    }
}
uint8_t parking_service_init(ParkingIdleCallback idle, ParkingDiagnosticsCallback diagnostics)
{
    is_idle = idle;
    print_diagnostics = diagnostics;
    sms_status = sms_phone[0] ? 1U : 0U;
    sms_queue = osMessageQueueNew(APP_SMS_QUEUE_LENGTH, sizeof(SmsMessage), NULL);
    if (!sms_queue) return 0U;
    gsm_thread = osThreadNew(gsm_task, NULL, &gsm_attributes);
    return gsm_thread != NULL;
}
uint8_t parking_service_sms_status(void) { return sms_status; }
void parking_service_notice(const ParkingContext *ctx, uint8_t kind)
{
    SmsMessage message;
    memset(&message, 0, sizeof(message));
    taskENTER_CRITICAL();
    memcpy(message.phone, sms_phone, sizeof(message.phone));
    taskEXIT_CRITICAL();
    if (!message.phone[0]) return;
    if (kind == 3U)
        snprintf(message.text, sizeof(message.text), "Parking FAULT code=%u pending=%u. Inspect before recovery.",
                 (unsigned)stepper_get_fault(), ctx->pending);
    else
        snprintf(message.text, sizeof(message.text),
                 "%s slot=%u UID=%02X%02X%02X%02X fee=%lu cents duration=%lu s",
                 kind == 1U ? "Parked" : "Retrieved", ctx->selected_slot,
                 ctx->current_uid[0], ctx->current_uid[1], ctx->current_uid[2], ctx->current_uid[3],
                 (unsigned long)(kind == 1U ? 0U : ctx->quote_cents),
                 (unsigned long)(kind == 1U ? 0U : ctx->quote_seconds));
    if (!sms_queue || osMessageQueuePut(sms_queue, &message, 0U, 0U) != osOK) {
        sms_drops++; sms_status = 3U; printf("[SMS] queue full, notification dropped\r\n");
    }
}
static uint8_t maintenance_ready(const ParkingContext *ctx)
{
    return ctx->state == PARK_FAULT && !sensor_emergency_active() && is_idle && is_idle();
}
static void status(const ParkingContext *ctx)
{
    unsigned i;
    printf("[STATUS] state=%u phase=%u fault=%u pending=%u check=%u data=%u rtc=%lu sms=%u\r\n",
        ctx->state, ctx->motion_phase, stepper_get_fault(), ctx->pending, ctx->reconcile_required,
        ctx->storage_fault, (unsigned long)bsp_clock_now(), sms_status);
    for (i = 0U; i < PARKING_SLOT_COUNT; i++)
        printf("[SLOT %u] occupied=%u UID=%02X%02X%02X%02X epoch=%lu uncertain=%u\r\n", i,
            ctx->slots[i].occupied, ctx->slots[i].uid[0], ctx->slots[i].uid[1],
            ctx->slots[i].uid[2], ctx->slots[i].uid[3],
            (unsigned long)ctx->slots[i].start_epoch, ctx->slots[i].time_uncertain);
    printf("[STATUS] fee=%lu cents seconds=%lu uncertain=%u sensors=0x%lx home=%u,%u pos=%ld,%ld\r\n",
        (unsigned long)ctx->quote_cents, (unsigned long)ctx->quote_seconds, ctx->quote_uncertain,
        (unsigned long)sensor_read_all(), stepper_is_homed(MOTOR_A), stepper_is_homed(MOTOR_B),
        (long)stepper_get_position(MOTOR_A), (long)stepper_get_position(MOTOR_B));
    printf("[DIAG] heap_free=%lu heap_min=%lu parking_stack_words=%lu sms_drop=%lu sms_fail=%lu rx_drop=%lu\r\n",
        (unsigned long)xPortGetFreeHeapSize(), (unsigned long)xPortGetMinimumEverFreeHeapSize(),
        (unsigned long)uxTaskGetStackHighWaterMark(NULL), (unsigned long)sms_drops,
        (unsigned long)sms_failures, (unsigned long)bsp_console_overflows());
    if (gsm_thread) printf("[STACK] gsm=%lu words\r\n",
        (unsigned long)uxTaskGetStackHighWaterMark(gsm_thread));
    if (print_diagnostics) print_diagnostics();
}
static void execute(ParkingContext *ctx, ParkingMoveCallback move, const ConsoleCommand *cmd)
{
    AppEvent event = {0};
    unsigned i;
    uint8_t ok = 0U;
    event.tick = HAL_GetTick();
    switch (cmd->type) {
    case CONSOLE_HELP:
        printf("HELP STATUS CARD 8hex CONFIRM PAY CANCEL STOP HOME TIME epoch PHONE number|OFF FEE cents SMS TEST\r\n"
               "RECON BEGIN -> SLOT 0..3 8hex|- (all four) -> RECON COMMIT -> HOME\r\n"
               "FORMAT CONFIRM: only FAULT+empty garage+all four staged empty; erases records!\r\n");
        return;
    case CONSOLE_STATUS: status(ctx); return;
    case CONSOLE_CARD:
        /* 联调入口仅允许空闲/满位；不能绕过故障、交接或缴费状态。 */
        if (ctx->state != PARK_IDLE && ctx->state != PARK_FULL) break;
        memcpy(event.uid, cmd->uid, 4U); event.type = APP_EVENT_CARD_DETECTED;
        parking_handle_event(ctx, &event, move); ok = 1U; break;
    case CONSOLE_PAY:
        if (ctx->state != PARK_WAIT_PAYMENT || ctx->quote_uncertain) break;
        event.type = APP_EVENT_KEY_LONG_PRESS;
        parking_handle_event(ctx, &event, move); ok = ctx->state == PARK_MOVING_OUT; break;
    case CONSOLE_CONFIRM:
        if (ctx->state != PARK_SELECT_ACTION) break;
        event.type = APP_EVENT_KEY_PRESS;
        parking_handle_event(ctx, &event, move); ok = ctx->state == PARK_MOVING_IN; break;
    case CONSOLE_CANCEL:
        if (ctx->state == PARK_WAIT_PAYMENT || ctx->state == PARK_SELECT_ACTION) {
            ctx->state = PARK_IDLE; ctx->quote_uncertain = 0U; ok = 1U;
        } break;
    case CONSOLE_STOP:
        event.type = APP_EVENT_MOTION_FAULT;
        parking_handle_event(ctx, &event, move); ok = 1U; break;
    case CONSOLE_HOME:
        if (!maintenance_ready(ctx) || ctx->reconcile_required || ctx->storage_fault) break;
        parking_begin_home(ctx, move, 1U); ok = ctx->state == PARK_MOVING_HOME; break;
    case CONSOLE_FEE:
        ok = parking_set_fee(ctx, cmd->value);
        if (ok) printf("[FEE] manual assessment=%lu cents; confirm with PAY/long press\r\n", (unsigned long)cmd->value);
        break;
    case CONSOLE_TIME:
        if (!is_idle || !is_idle() || ctx->pending || ctx->state == PARK_SELECT_ACTION ||
            ctx->state == PARK_WAIT_PAYMENT) break;
        /* 防止修改正在停车车辆的时间基准；有车时通过FEE人工核费。 */
        for (i = 0U; i < PARKING_SLOT_COUNT; i++) if (ctx->slots[i].occupied) break;
        if (i == PARKING_SLOT_COUNT) ok = bsp_clock_set(cmd->value);
        break;
    case CONSOLE_PHONE:
        taskENTER_CRITICAL(); memcpy(sms_phone, cmd->phone, sizeof(sms_phone)); taskEXIT_CRITICAL();
        sms_status = sms_phone[0] ? 1U : 0U; ok = 1U; break;
    case CONSOLE_SMS_TEST: {
        SmsMessage message = {{0}, "Smart Parking SMS test. Local control remains available."};
        taskENTER_CRITICAL(); memcpy(message.phone, sms_phone, sizeof(sms_phone)); taskEXIT_CRITICAL();
        if (message.phone[0] && sms_queue)
            ok = osMessageQueuePut(sms_queue, &message, 0U, 0U) == osOK;
        break;
    }
    case CONSOLE_RECON_BEGIN:
        if (!maintenance_ready(ctx)) break;
        memset(staged, 0, sizeof(staged)); staged_mask = 0U; staging = 1U; ok = 1U; break;
    case CONSOLE_RECON_SLOT:
        if (!staging || !maintenance_ready(ctx)) break;
        memset(&staged[cmd->value], 0, sizeof(ParkingSlot));
        staged[cmd->value].occupied = cmd->occupied;
        memcpy(staged[cmd->value].uid, cmd->uid, 4U);
        staged[cmd->value].time_uncertain = cmd->occupied;
        staged_mask |= (uint8_t)(1U << cmd->value); ok = 1U; break;
    case CONSOLE_RECON_COMMIT:
        if (!staging || staged_mask != 15U || !maintenance_ready(ctx)) break;
        ok = parking_reconcile(ctx, staged);
        if (ok) staging = 0U;
        break;
    case CONSOLE_FORMAT:
        /* 损坏日志只能在人工确认整个车库已清空后重建。 */
        if (!staging || staged_mask != 15U || !maintenance_ready(ctx)) break;
        for (i = 0U; i < PARKING_SLOT_COUNT; i++)
            if (staged[i].occupied || (PARKING_SLOTS[i].occupancy_sensor < SENSOR_ID_COUNT &&
                sensor_read(PARKING_SLOTS[i].occupancy_sensor))) break;
        if (i != PARKING_SLOT_COUNT) break;
        if (!parking_store_format()) { ctx->storage_fault = ctx->reconcile_required = 1U; break; }
        ok = parking_reconcile(ctx, staged);
        if (ok) staging = 0U;
        break;
    default: break;
    }
    printf("[CMD] %s state=%u\r\n", ok ? "OK" : "REJECTED (check STATUS/HELP)", ctx->state);
}
void parking_service_poll(ParkingContext *ctx, ParkingMoveCallback move)
{
    static char line[APP_CONSOLE_LINE_BYTES];
    static unsigned length;
    static uint8_t discard;
    static uint32_t overflows;
    char ch;
    unsigned budget = 128U;
    if (overflows != bsp_console_overflows()) {
        overflows = bsp_console_overflows(); length = 0U; discard = 1U;
    }
    while (budget-- && bsp_console_get(&ch)) {
        if (overflows != bsp_console_overflows()) {
            overflows = bsp_console_overflows(); length = 0U; discard = 1U;
        }
        if (ch == '\r' || ch == '\n') {
            if (!discard && length) {
                ConsoleCommand command;
                line[length] = '\0';
                if (console_parse(line, &command)) execute(ctx, move, &command);
                else printf("[CMD] invalid; HELP\r\n");
            }
            length = 0U; discard = 0U;
        } else if (!discard) {
            if (ch == '\b' || ch == 127) { if (length) length--; }
            else if ((unsigned char)ch < 32U || length + 1U >= sizeof(line)) {
                discard = 1U; length = 0U; printf("[CMD] line rejected\r\n");
            } else line[length++] = ch;
        }
    }
}
