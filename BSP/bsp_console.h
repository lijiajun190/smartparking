#ifndef BSP_CONSOLE_H
#define BSP_CONSOLE_H
#include <stdint.h>
void bsp_console_init(void);
int bsp_console_write(const char *text, int length);
uint8_t bsp_console_get(char *character);
uint32_t bsp_console_overflows(void);
#endif
