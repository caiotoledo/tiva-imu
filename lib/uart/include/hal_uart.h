#ifndef HAL_UART_H_
#define HAL_UART_H_

/**
 * @brief Enable UART asynchronous
 * 
 */
#define UART_BUFFERED
#include "utils/uartstdio.h"

/**
 * @brief Enum definition for UART Interface
 * 
 */
typedef enum {
    UART0,
} eUART_BASE;

/**
 * @brief Enable UART Interface
 * 
 * @param uart UART Interface
 * @param baudrate Baudrate in bps
 */
void UART_Enable(eUART_BASE uart, uint32_t baudrate);

#endif /* HAL_UART_H_ */
