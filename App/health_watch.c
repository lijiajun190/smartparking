#include "health_watch.h"
static volatile uint32_t beats[HEALTH_COUNT];
static uint32_t failed;
void health_init(uint32_t now)
{
    unsigned i;
    failed = 0U;
    for (i = 0U; i < HEALTH_COUNT; i++) beats[i] = now;
}
void health_beat(HealthTask task, uint32_t now)
{
    if ((unsigned)task < HEALTH_COUNT) beats[task] = now;
}
uint8_t health_check(uint32_t now)
{
    unsigned i;
    for (i = 0U; i < HEALTH_COUNT; i++) {
        /* 先读取样本：任务可能在读取now之后抢占并写入更晚的tick。
         * 有符号差避免把这几毫秒误判为uint32回绕后的超时。 */
        uint32_t beat = beats[i];
        if ((int32_t)(now - beat) > (int32_t)HEALTH_TIMEOUT_MS) failed |= 1UL << i;
    }
    return failed == 0U;
}
uint32_t health_failed_mask(void) { return failed; }
