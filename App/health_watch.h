#ifndef HEALTH_WATCH_H
#define HEALTH_WATCH_H
#include <stdint.h>
typedef enum { HEALTH_SAFETY, HEALTH_MOTION, HEALTH_PARKING, HEALTH_INPUT,
               HEALTH_UI, HEALTH_RFID, HEALTH_GSM, HEALTH_COUNT } HealthTask;
#define HEALTH_TIMEOUT_MS 2500U
/* 先init再启动调度；Motion在每步完成后报告，其余任务在主循环报告。 */
void health_init(uint32_t now);
void health_beat(HealthTask task, uint32_t now);
uint8_t health_check(uint32_t now);
uint32_t health_failed_mask(void);
#endif
