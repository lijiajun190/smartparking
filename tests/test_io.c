/* Production console/service/modem integration with deterministic UART/RTOS mocks. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_console.h"
#include "bsp_sim800.h"
#include "bsp_sensors.h"
#include "bsp_clock.h"
#include "parking_service.h"
#include "parking_store.h"
#include "health_watch.h"

#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
UART_HandleTypeDef huart1 = {USART1}, huart2 = {USART2};
DMA_HandleTypeDef hdma_usart2_rx;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
static uint32_t now, epoch, sensors, irq, max_heartbeat_gap, last_heartbeat;
static uint8_t *console_byte, *dma_bytes;
static unsigned dma_position, rx_event, rx_starts, tx_calls, tx_fail, lock_fail;
static unsigned acquired, released, move_count, format_count, idle = 1U;
static MotorFault fault;
static char response[800], command[160], sent_phone[32];
static unsigned response_at, response_length, burst = 3U;
static unsigned silence, network = 1U, receive_error, dma_start_fail, sms_count, at_count, escaped;
static unsigned no_final_ok, response_delay, fail_body, fail_end;
static void (*worker)(void *);
static unsigned queue_size, queue_length, queue_count, queue_head, queue_tail, queue_rejections;
static uint8_t queue_data[8][200];
static jmp_buf worker_done;
static uint8_t worker_running;

uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return irq; }
uint32_t __get_IPSR(void) { return 0U; }
void __disable_irq(void) { irq = 1U; }
void __set_PRIMASK(uint32_t mask) { irq = mask; }
void Error_Handler(void) { CHECK(0); }
void health_beat(HealthTask task, uint32_t tick)
{
    if (task != HEALTH_GSM) return;
    if (tick - last_heartbeat > max_heartbeat_gap) max_heartbeat_gap = tick - last_heartbeat;
    last_heartbeat = tick;
}
static void reply(const char *text)
{
    CHECK(strlen(text) < sizeof(response));
    strcpy(response, text); response_at = 0U; response_length = (unsigned)strlen(text);
}
void osDelay(uint32_t delay)
{
    unsigned count;
    now += delay;
    if (response_delay) { response_delay--; return; }
    if (receive_error) { receive_error = 0U; HAL_UART_ErrorCallback(&huart2); return; }
    if (!dma_bytes) return;
    count = burst;
    while (count-- && response_at < response_length) {
        dma_bytes[dma_position++] = (uint8_t)response[response_at++];
        if (dma_position == GSM_RX_BUFFER_SIZE) {
            dma_position = 0U; hdma_usart2_rx.remaining = GSM_RX_BUFFER_SIZE;
            rx_event = HAL_UART_RXEVENT_TC;
            HAL_UARTEx_RxEventCallback(&huart2, GSM_RX_BUFFER_SIZE);
        } else hdma_usart2_rx.remaining = GSM_RX_BUFFER_SIZE - dma_position;
    }
    rx_event = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart2, (uint16_t)dma_position);
}
void HAL_Delay(uint32_t delay) { osDelay(delay); }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size)
{
    CHECK(uart == &huart2 && size == GSM_RX_BUFFER_SIZE);
    rx_starts++; dma_bytes = bytes; dma_position = 0U; hdma_usart2_rx.remaining = size;
    response_at = response_length = 0U;
    return dma_start_fail ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
unsigned HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart) { (void)uart; return rx_event; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t length)
{ CHECK(uart == &huart1 && length == 1U); console_byte = bytes; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t length, uint32_t timeout)
{
    (void)timeout; tx_calls++;
    if (tx_fail) return HAL_ERROR;
    if (uart == &huart1) return HAL_OK;
    CHECK(uart == &huart2);
    if (length == 1U && bytes[0] == 27U) { escaped++; return HAL_OK; }
    if (length == 1U && bytes[0] == 26U) {
        if (fail_end) { fail_end = 0U; return HAL_ERROR; }
        sms_count++;
        if (!silence) reply(no_final_ok ? "\r\n+CMGS: 17\r\n" : "\r\n+CMGS: 17\r\n\r\nOK\r\n");
        return HAL_OK;
    }
    if (length < 2U || bytes[0] != 'A' || bytes[1] != 'T') {
        if (fail_body) { fail_body = 0U; return HAL_ERROR; }
        return HAL_OK;
    }
    CHECK(length < sizeof(command)); memcpy(command, bytes, length); command[length] = '\0';
    if (!strcmp(command, "AT\r\n")) at_count++;
    if (silence) return HAL_OK;
    if (strstr(command, "AT+CMGS=")) {
        const char *start = strchr(command, '"'); const char *end = strrchr(command, '"');
        CHECK(start && end && end > start && end - start < sizeof(sent_phone));
        memcpy(sent_phone, start + 1, end - start - 1); sent_phone[end - start - 1] = '\0';
        reply("\r\n> ");
    } else if (strstr(command, "AT+CPIN?")) reply("\r\n+CPIN: READY\r\n\r\nOK\r\n");
    else if (strstr(command, "AT+CSQ")) reply("\r\n+CSQ: 23,0\r\n\r\nOK\r\n");
    else if (strstr(command, "AT+CREG?")) reply(network ? "\r\n+CREG: 2,5\r\n\r\nOK\r\n" : "\r\n+CREG: 2,0\r\n\r\nOK\r\n");
    else if (strstr(command, "AT+ERROR")) reply("\r\nERROR\r\n");
    else if (strstr(command, "AT+INCOMPLETE")) reply("\r\n+VALUE: 12\r\n");
    else if (strstr(command, "AT+OVERFLOW")) {
        memset(response, 'x', 600U); response[600] = '\0'; response_length = 600U; response_at = 0U;
    } else reply("\r\nOK\r\n");
    return HAL_OK;
}
osMutexId_t osMutexNew(const void *attr) { (void)attr; return (void *)1; }
int osMutexAcquire(osMutexId_t mutex, uint32_t delay)
{ (void)mutex; CHECK(delay == 50U); if (lock_fail) return osError; acquired++; return osOK; }
int osMutexRelease(osMutexId_t mutex) { (void)mutex; released++; return osOK; }
int xTaskGetSchedulerState(void) { return 1; }
size_t xPortGetFreeHeapSize(void) { return 10000U; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 9000U; }
unsigned uxTaskGetStackHighWaterMark(void *task) { (void)task; return 600U; }
osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const void *attr)
{
    (void)attr; CHECK(count <= 8U && size <= 200U);
    queue_size = size; queue_length = count; queue_count = queue_head = queue_tail = 0U;
    return (void *)2;
}
int osMessageQueuePut(osMessageQueueId_t queue, const void *message, uint8_t priority, uint32_t timeout)
{
    (void)queue; (void)priority; CHECK(timeout == 0U);
    if (queue_count == queue_length) { queue_rejections++; return osError; }
    memcpy(queue_data[queue_head], message, queue_size);
    queue_head = (queue_head + 1U) % queue_length; queue_count++; return osOK;
}
int osMessageQueueGet(osMessageQueueId_t queue, void *message, void *priority, uint32_t timeout)
{
    (void)queue; (void)priority; (void)timeout;
    if (!queue_count) { CHECK(worker_running); longjmp(worker_done, 1); }
    memcpy(message, queue_data[queue_tail], queue_size);
    queue_tail = (queue_tail + 1U) % queue_length; queue_count--; return osOK;
}
osThreadId_t osThreadNew(void (*function)(void *), void *arg, const osThreadAttr_t *attr)
{ (void)arg; CHECK(attr->stack_size >= 4096U); worker = function; return (void *)3; }
uint32_t bsp_clock_now(void) { return epoch; }
uint8_t bsp_clock_set(uint32_t value) { epoch = value; return 1U; }
uint8_t sensor_read(SensorId id) { return (sensors & SENSOR_FLAG(id)) != 0U; }
uint32_t sensor_read_all(void) { return sensors; }
uint8_t sensor_emergency_active(void) { return sensor_read(SENSOR_EMERGENCY); }
MotorFault stepper_get_fault(void) { return fault; }
void stepper_request_stop(MotorFault value) { fault = value; }
MotorStatus stepper_get_status(MotorID motor) { (void)motor; return MOTOR_IDLE; }
uint8_t stepper_is_homed(MotorID motor) { (void)motor; return 1U; }
int32_t stepper_get_position(MotorID motor) { (void)motor; return 0; }
uint8_t parking_store_format(void) { format_count++; return 1U; }
static uint8_t persisted(const ParkingContext *ctx) { (void)ctx; return 1U; }
static uint8_t is_idle(void) { return (uint8_t)idle; }
static uint8_t move(ParkingMotionType type, MotorID motor, uint32_t steps, MotorDirection direction, uint32_t id)
{ (void)type; (void)motor; (void)steps; (void)direction; CHECK(id); move_count++; return 1U; }
static void init_context(ParkingContext *ctx)
{ parking_init(ctx); ctx->state = PARK_IDLE; ctx->persist = persisted; ctx->clock = bsp_clock_now; fault = MOTOR_FAULT_NONE; }
static void inject(const char *text)
{ while (*text) { CHECK(console_byte); *console_byte = (uint8_t)*text++; HAL_UART_RxCpltCallback(&huart1); } }
static void poll(ParkingContext *ctx) { parking_service_poll(ctx, move); }
static void run(ParkingContext *ctx, const char *text) { inject(text); poll(ctx); poll(ctx); poll(ctx); }
static void run_worker(void)
{
    worker_running = 1U; last_heartbeat = now; max_heartbeat_gap = 0U;
    if (setjmp(worker_done) == 0) worker(NULL);
    worker_running = 0U;
    /* Retry delay is 500ms plus the final 10ms response polling slice. */
    CHECK(max_heartbeat_gap <= 600U);
}

static void test_console_and_service(void)
{
    ParkingContext ctx;
    unsigned before, i;
    init_context(&ctx); bsp_console_init(); CHECK(parking_service_init(is_idle, NULL));
    CHECK(bsp_console_write("hello", 5) == 5); CHECK(acquired == released);
    lock_fail = 1U; before = tx_calls; CHECK(bsp_console_write("hello", 5) == -1); CHECK(tx_calls == before); lock_fail = 0U;
    tx_fail = 1U; CHECK(bsp_console_write("hello", 5) == -1); CHECK(acquired == released); tx_fail = 0U;
    inject("CARD 0102"); poll(&ctx); CHECK(ctx.state == PARK_IDLE);
    run(&ctx, "0304\n"); CHECK(ctx.state == PARK_SELECT_ACTION);
    run(&ctx, "CANCEL\n"); CHECK(ctx.state == PARK_IDLE);
    inject("CARD 0102"); poll(&ctx);
    for (i = 0U; i < 300U; i++) inject("x");
    run(&ctx, "0304\n"); CHECK(ctx.state == PARK_IDLE); CHECK(bsp_console_overflows());
    inject("CARD 0102"); poll(&ctx); HAL_UART_ErrorCallback(&huart1);
    run(&ctx, "0304\n"); CHECK(ctx.state == PARK_IDLE);
    for (i = 0U; i < 100U; i++) { inject("x"); poll(&ctx); }
    run(&ctx, "\nCARD 01020304\nCONFIRM\n"); CHECK(ctx.state == PARK_MOVING_IN); CHECK(ctx.pending == 1U);
    before = move_count; run(&ctx, "CARD 05060708\nPAY\nHOME\n"); CHECK(move_count == before);
    run(&ctx, "STOP\n"); CHECK(ctx.state == PARK_FAULT && ctx.reconcile_required);
    before = move_count; run(&ctx, "HOME\n"); CHECK(move_count == before);
    printf("PASS console framing, overflow/error recovery, mutex, command guards\n");
}
static void test_maintenance(void)
{
    ParkingContext ctx;
    unsigned before;
    init_context(&ctx); ctx.state = PARK_FAULT; ctx.pending = 1U; ctx.reconcile_required = 1U;
    before = move_count;
    sensors = SENSOR_FLAG(SENSOR_EMERGENCY); run(&ctx, "RECON BEGIN\nHOME\n"); CHECK(move_count == before);
    sensors = 0U;
    run(&ctx, "RECON BEGIN\nSLOT 0 -\nSLOT 1 -\nSLOT 2 11223344\nRECON COMMIT\n"); CHECK(ctx.pending);
    run(&ctx, "SLOT 3 11223344\nRECON COMMIT\n"); CHECK(ctx.pending);
    run(&ctx, "SLOT 3 -\nRECON COMMIT\n"); CHECK(!ctx.pending && !ctx.reconcile_required); CHECK(ctx.slots[2].occupied && ctx.slots[2].time_uncertain);
    epoch = 0U; run(&ctx, "TIME 1800000000\n"); CHECK(epoch == 0U);
    run(&ctx, "HOME\n"); CHECK(ctx.state == PARK_MOVING_HOME && move_count == before + 1U);
    init_context(&ctx); ctx.state = PARK_FAULT; ctx.storage_fault = ctx.reconcile_required = 1U;
    run(&ctx, "RECON BEGIN\nSLOT 0 -\nSLOT 1 -\nSLOT 2 -\nSLOT 3 -\n");
    sensors = SENSOR_FLAG(SENSOR_IR_1); before = format_count;
    run(&ctx, "FORMAT CONFIRM\n"); CHECK(format_count == before);
    sensors = 0U; idle = 0U; run(&ctx, "FORMAT CONFIRM\n"); CHECK(format_count == before);
    idle = 1U; run(&ctx, "FORMAT CONFIRM\n"); CHECK(format_count == before + 1U); CHECK(!ctx.storage_fault && !ctx.reconcile_required && ctx.state == PARK_FAULT);
    run(&ctx, "TIME 1800000000\n"); CHECK(epoch == 1800000000U);
    printf("PASS maintenance: emergency/idle, four slots, duplicate UID, format/clock guards\n");
}
static void test_payment(void)
{
    ParkingContext ctx;
    unsigned before;
    init_context(&ctx); ctx.slots[2].occupied = ctx.slots[2].time_uncertain = 1U;
    ctx.slots[2].uid[0] = 0x11; ctx.slots[2].uid[1] = 0x22; ctx.slots[2].uid[2] = 0x33; ctx.slots[2].uid[3] = 0x44;
    run(&ctx, "CARD 11223344\n"); CHECK(ctx.state == PARK_WAIT_PAYMENT && ctx.quote_uncertain);
    before = move_count; run(&ctx, "PAY\n"); CHECK(move_count == before);
    run(&ctx, "FEE 1250\n"); CHECK(!ctx.quote_uncertain && ctx.quote_cents == 1250U);
    run(&ctx, "PAY\n"); CHECK(ctx.state == PARK_MOVING_OUT && move_count > before);
    printf("PASS uncertain time requires manual fee before payment\n");
}
static void test_gsm(void)
{
    unsigned before;
    uint32_t started;
    network = 0U; CHECK(gsm_init() == GSM_NO_NETWORK);
    network = 1U; CHECK(gsm_init() == GSM_OK);
    started = now; CHECK(gsm_send_command("AT+CSQ", "+CSQ:", 500U) == GSM_OK); CHECK(now - started >= 70U);
    CHECK(gsm_send_command("AT+INCOMPLETE", "+VALUE:", 100U) == GSM_TIMEOUT);
    CHECK(gsm_send_command("AT+ERROR", NULL, 100U) == GSM_ERROR);
    tx_fail = 1U; CHECK(gsm_send_command("AT", NULL, 100U) == GSM_ERROR); tx_fail = 0U;
    dma_start_fail = 1U; CHECK(gsm_send_command("AT", NULL, 100U) == GSM_ERROR); dma_start_fail = 0U;
    receive_error = 1U; CHECK(gsm_send_command("AT", NULL, 100U) == GSM_ERROR);
    CHECK(gsm_send_command("AT", NULL, 100U) == GSM_OK);
    burst = 600U; CHECK(gsm_send_command("AT+OVERFLOW", NULL, 100U) == GSM_ERROR);
    burst = 3U; CHECK(gsm_send_command("AT", NULL, 100U) == GSM_OK);
    before = tx_calls; CHECK(gsm_send_sms("123\"\r", "test") == GSM_ERROR); CHECK(tx_calls == before);
    CHECK(gsm_send_sms("+8613800138000", "integration test") == GSM_OK);
    before = escaped; fail_body = 1U;
    CHECK(gsm_send_sms("13800138000", "test") == GSM_ERROR); CHECK(escaped == before + 1U);
    before = escaped; fail_end = 1U;
    CHECK(gsm_send_sms("13800138000", "test") == GSM_ERROR); CHECK(escaped == before + 1U);
    no_final_ok = 1U; CHECK(gsm_send_sms("13800138000", "test") == GSM_TIMEOUT); no_final_ok = 0U;
    silence = 1U; before = escaped; CHECK(gsm_send_sms("13800138000", "test") == GSM_TIMEOUT); CHECK(escaped == before + 1U); silence = 0U;
    CHECK(gsm_send_sms("13800138000", "recovered") == GSM_OK);
    printf("PASS modem fragmented responses, registration, timeout, DMA overrun, UART recovery, SMS\n");
}
static void test_notifications(void)
{
    ParkingContext ctx;
    unsigned before, i;
    init_context(&ctx); queue_count = queue_head = queue_tail = 0U;
    run(&ctx, "PHONE 13800138000\nSMS TEST\nPHONE 13900139000\n"); CHECK(queue_count == 1U);
    before = sms_count; run_worker(); CHECK(sms_count == before + 1U); CHECK(!strcmp(sent_phone, "13800138000")); CHECK(parking_service_sms_status() == 2U);
    for (i = 0U; i < 9U; i++) parking_service_notice(&ctx, 1U);
    CHECK(queue_count == 8U && queue_rejections == 1U); CHECK(ctx.state == PARK_IDLE);
    run_worker(); CHECK(queue_count == 0U);
    run(&ctx, "SMS TEST\n"); silence = 1U; before = at_count; run_worker(); silence = 0U;
    CHECK(at_count == before + 3U); CHECK(parking_service_sms_status() == 3U);
    run(&ctx, "PHONE OFF\nSMS TEST\n"); CHECK(queue_count == 0U && parking_service_sms_status() == 0U);
    printf("PASS asynchronous SMS snapshot, bounded queue, three retries, failure isolation\n");
}
int main(void)
{
    test_console_and_service(); test_maintenance(); test_payment(); test_gsm(); test_notifications();
    printf("5 IO integration groups passed\n"); return 0;
}
