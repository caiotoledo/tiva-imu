#include <FreeRTOS.h>
#include <timers.h>

#include "timer.h"

uint32_t GetMillisFromISR(void)
{
    return (xTaskGetTickCountFromISR()/portTICK_RATE_MS);
}

uint32_t GetMillis(void)
{
    return (xTaskGetTickCount()/portTICK_RATE_MS);
}
