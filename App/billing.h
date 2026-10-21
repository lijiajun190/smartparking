#ifndef BILLING_H
#define BILLING_H
#include <stdint.h>
typedef struct { uint32_t unit_seconds, rate_cents; } BillingConfig;
uint32_t billing_elapsed_seconds(uint32_t start_tick, uint32_t end_tick);
uint32_t billing_calculate_cents(uint32_t seconds, const BillingConfig *config);
/* RTC使用2000~2099年，时间戳采用Unix秒；非法日期返回0。 */
uint32_t billing_epoch(unsigned year, unsigned month, unsigned day,
                       unsigned hour, unsigned minute, unsigned second);
uint8_t billing_calendar(uint32_t epoch, unsigned *year, unsigned *month,
                          unsigned *day, unsigned *hour, unsigned *minute, unsigned *second);
#endif
