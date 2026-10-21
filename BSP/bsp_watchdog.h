#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H
void bsp_watchdog_init(void);
void bsp_watchdog_refresh(void);
/* 致命故障专用：关中断、直接去励磁并等待IWDG复位；不依赖RTOS/堆栈业务。 */
void bsp_runtime_panic(void);
#endif
