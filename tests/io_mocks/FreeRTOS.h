#include <stdint.h>
#include <stddef.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
