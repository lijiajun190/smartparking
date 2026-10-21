#ifndef IO_TEST_CMSIS_H
#define IO_TEST_CMSIS_H
#include <stdint.h>
typedef void *osMutexId_t;
typedef void *osMessageQueueId_t;
typedef void *osThreadId_t;
typedef struct { const char *name; uint32_t stack_size; int priority; } osThreadAttr_t;
enum { osOK, osError, osPriorityBelowNormal };
void osDelay(uint32_t);
osMutexId_t osMutexNew(const void *);
int osMutexAcquire(osMutexId_t, uint32_t);
int osMutexRelease(osMutexId_t);
osMessageQueueId_t osMessageQueueNew(uint32_t, uint32_t, const void *);
int osMessageQueueGet(osMessageQueueId_t, void *, void *, uint32_t);
int osMessageQueuePut(osMessageQueueId_t, const void *, uint8_t, uint32_t);
osThreadId_t osThreadNew(void (*)(void *), void *, const osThreadAttr_t *);
#endif
