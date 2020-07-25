#ifndef HAL_UART_H_
#define HAL_UART_H_

#include <string.h>

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

/**
 * @brief Get a line string from the UART (Blocking call)
 *
 * @param str Pointer buffer to store the string
 * @param size Input Size of the str buffer / Output string size received
 * @return int Return 0 successful
 */
int UART_GetLine(char *str, size_t *size);

#endif /* HAL_UART_H_ */
