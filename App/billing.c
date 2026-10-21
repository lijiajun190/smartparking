#include "billing.h"
#include <stddef.h>
static unsigned month_days(unsigned year, unsigned month)
{
    static const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    return days[month - 1U] + (month == 2U && year % 4U == 0U ? 1U : 0U);
}
uint32_t billing_elapsed_seconds(uint32_t start_tick, uint32_t end_tick)
{ return (uint32_t)(end_tick - start_tick) / 1000U; }
uint32_t billing_calculate_cents(uint32_t seconds, const BillingConfig *config)
{
    uint64_t units, cents;
    if (!config || !config->unit_seconds) return UINT32_MAX;
    units = seconds / config->unit_seconds + (seconds % config->unit_seconds != 0U);
    cents = units * config->rate_cents;
    return cents > UINT32_MAX ? UINT32_MAX : (uint32_t)cents;
}
uint32_t billing_epoch(unsigned year, unsigned month, unsigned day,
                       unsigned hour, unsigned minute, unsigned second)
{
    unsigned y, m;
    uint32_t days = 10957U; /* 1970-01-01 -> 2000-01-01 */
    if (year < 2000U || year > 2099U || month < 1U || month > 12U ||
        day < 1U || day > month_days(year, month) || hour > 23U || minute > 59U || second > 59U)
        return 0U;
    for (y = 2000U; y < year; y++) days += y % 4U == 0U ? 366U : 365U;
    for (m = 1U; m < month; m++) days += month_days(year, m);
    return (days + day - 1U) * 86400U + hour * 3600U + minute * 60U + second;
}
uint8_t billing_calendar(uint32_t epoch, unsigned *year, unsigned *month,
                          unsigned *day, unsigned *hour, unsigned *minute, unsigned *second)
{
    uint32_t days;
    unsigned y = 2000U, m = 1U, span;
    if (!year || !month || !day || !hour || !minute || !second ||
        epoch < 946684800U || epoch > 4102444799U) return 0U;
    days = epoch / 86400U - 10957U;
    while ((span = (y % 4U == 0U ? 366U : 365U)), days >= span) { days -= span; y++; }
    while (days >= month_days(y, m)) { days -= month_days(y, m); m++; }
    *year = y; *month = m; *day = days + 1U;
    *hour = epoch % 86400U / 3600U; *minute = epoch % 3600U / 60U; *second = epoch % 60U;
    return 1U;
}
