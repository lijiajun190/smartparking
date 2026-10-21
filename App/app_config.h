#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 联调默认值：30分钟500分；0秒0分，不足周期向上取整。
 * 短信号码默认留空，可用串口 PHONE 命令配置本次运行的通知接收号码。 */
#define APP_BILLING_UNIT_SECONDS 1800U
#define APP_BILLING_RATE_CENTS 500U
#define APP_SELECTION_TIMEOUT_MS 30000U
#define APP_SMS_PHONE ""
#define APP_SMS_MAX_ATTEMPTS 3U
#define APP_RTC_USE_LSE 0 /* 无32.768kHz晶振时使用LSI；跨整机掉电须LSE+VBAT */
#define APP_CONSOLE_LINE_BYTES 96U
#define APP_SMS_QUEUE_LENGTH 8U
#endif
