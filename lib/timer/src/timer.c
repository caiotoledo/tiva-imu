#include <FreeRTOS.h>
#include <timers.h>

#include "timer.h"

uint32_t GetMillis(void)
{
    return (xTaskGetTickCountFromISR()/portTICK_RATE_MS);
}
