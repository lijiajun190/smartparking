#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H
#include <stdint.h>
void bsp_clock_init(void);
uint32_t bsp_clock_now(void); /* 0表示未校时/不可信，不得跨掉电计费 */
uint8_t bsp_clock_set(uint32_t epoch);
#endif
