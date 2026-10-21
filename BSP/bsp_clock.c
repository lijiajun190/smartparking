#include "main.h"
#include "bsp_clock.h"
#include "billing.h"
#include "app_config.h"
static RTC_HandleTypeDef clock_rtc;
static uint8_t available;
#define CLOCK_MAGIC 0x434C4B32U

void bsp_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};
    uint8_t cold = __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) || __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST);
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    osc.OscillatorType = APP_RTC_USE_LSE ? RCC_OSCILLATORTYPE_LSE : RCC_OSCILLATORTYPE_LSI;
    osc.LSEState = APP_RTC_USE_LSE ? RCC_LSE_ON : RCC_LSE_OFF;
    osc.LSIState = RCC_LSI_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return;
    periph.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    periph.RTCClockSelection = APP_RTC_USE_LSE ? RCC_RTCCLKSOURCE_LSE : RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) return;
    __HAL_RCC_RTC_ENABLE();
    clock_rtc.Instance = RTC;
    clock_rtc.Init.HourFormat = RTC_HOURFORMAT_24;
    clock_rtc.Init.AsynchPrediv = 127U;
    clock_rtc.Init.SynchPrediv = APP_RTC_USE_LSE ? 255U : 249U;
    clock_rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    clock_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    clock_rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&clock_rtc) != HAL_OK) return;
    available = 1U;
    /* LSI不由VBAT维持，冷启动即使保留了魔数也不可信。 */
    if (!APP_RTC_USE_LSE && cold) HAL_RTCEx_BKUPWrite(&clock_rtc, RTC_BKP_DR0, 0U);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}
uint32_t bsp_clock_now(void)
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    if (!available || HAL_RTCEx_BKUPRead(&clock_rtc, RTC_BKP_DR0) != CLOCK_MAGIC) return 0U;
    if (HAL_RTC_GetTime(&clock_rtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_GetDate(&clock_rtc, &date, RTC_FORMAT_BIN) != HAL_OK) return 0U;
    return billing_epoch(2000U + date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
}
uint8_t bsp_clock_set(uint32_t epoch)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    unsigned y, m, d, h, min, s;
    if (!available || !billing_calendar(epoch, &y, &m, &d, &h, &min, &s)) return 0U;
    HAL_RTCEx_BKUPWrite(&clock_rtc, RTC_BKP_DR0, 0U);
    time.Hours = (uint8_t)h; time.Minutes = (uint8_t)min; time.Seconds = (uint8_t)s;
    date.Year = (uint8_t)(y - 2000U); date.Month = (uint8_t)m; date.Date = (uint8_t)d;
    date.WeekDay = (uint8_t)((epoch / 86400U + 3U) % 7U + 1U);
    if (HAL_RTC_SetTime(&clock_rtc, &time, RTC_FORMAT_BIN) != HAL_OK ||
        HAL_RTC_SetDate(&clock_rtc, &date, RTC_FORMAT_BIN) != HAL_OK) return 0U;
    HAL_RTCEx_BKUPWrite(&clock_rtc, RTC_BKP_DR0, CLOCK_MAGIC);
    return 1U;
}
