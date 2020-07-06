#ifndef TIMERS_H
#define TIMERS_H

#include <FreeRTOS.h>

TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);

#endif /* TIMERS_H */
