#ifndef IO_TEST_MAIN_H
#define IO_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { unsigned id; } GPIO_TypeDef;
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY} HAL_StatusTypeDef;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
#define USART1 ((void *)1)
#define USART2 ((void *)2)
#define DMA_IT_HT 1U
#define HAL_UART_RXEVENT_IDLE 0U
#define HAL_UART_RXEVENT_TC 1U
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define __HAL_DMA_DISABLE_IT(dma, flag) ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)0)
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
unsigned HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void Error_Handler(void);
uint32_t __get_PRIMASK(void);
uint32_t __get_IPSR(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
#endif
