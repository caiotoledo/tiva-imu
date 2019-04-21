#ifndef HAL_UART_UTILS_H_
#define HAL_UART_UTILS_H_

#include "utils/uartstdio.h"

typedef enum {
    UART0,
} eUART_BASE;

void UART_Enable(eUART_BASE uart, uint32_t baudrate);

#endif /* HAL_UART_UTILS_H_ */
