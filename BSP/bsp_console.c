#include "usart.h"
#include "bsp_console.h"
#include "bsp_sim800.h"
#include "cmsis_os.h"
/* USART1单字节中断写入，ParkingTask读取；溢出时丢弃整段直到换行。
 * 缓冲区已满且主任务没读到换行时，主任务检测overflow计数并丢弃当前命令。 */
static uint8_t byte, ring[256];
static volatile uint16_t head, tail;
static volatile uint32_t overflows;
static uint8_t discarding;
static osMutexId_t tx_mutex;
void bsp_console_init(void)
{
    tx_mutex = osMutexNew(NULL);
    if (!tx_mutex) Error_Handler();
    if (HAL_UART_Receive_IT(&huart1, &byte, 1U) != HAL_OK) Error_Handler();
}
int bsp_console_write(const char *text, int length)
{
    HAL_StatusTypeDef result;
    osMutexId_t mutex = tx_mutex;
    if (__get_IPSR() || length < 0 || length > 65535) return -1;
    if (mutex && osMutexAcquire(mutex, 50U) != osOK) return -1;
    result = HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)length, 200U);
    if (mutex) (void)osMutexRelease(mutex);
    return result == HAL_OK ? length : -1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (uart->Instance != USART1) return;
    next = (head + 1U) & 255U;
    if (next == tail) { overflows++; discarding = 1U; head = tail = 0U; }
    else if (discarding) {
        if (byte == '\n' || byte == '\r') { ring[head] = '\n'; head = next; discarding = 0U; }
    } else { ring[head] = byte; head = next; }
    (void)HAL_UART_Receive_IT(&huart1, &byte, 1U);
}
uint8_t bsp_console_get(char *character)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t available = 0U;
    __disable_irq();
    if (head != tail) {
        *character = (char)ring[tail]; tail = (tail + 1U) & 255U; available = 1U;
    }
    __set_PRIMASK(mask);
    return available;
}
uint32_t bsp_console_overflows(void) { return overflows; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) {
        overflows++; head = tail = 0U; discarding = 1U;
        (void)HAL_UART_AbortReceive(uart);
        __HAL_UART_CLEAR_OREFLAG(uart);
        (void)HAL_UART_Receive_IT(uart, &byte, 1U);
    } else if (uart->Instance == USART2) gsm_receive_error();
}
