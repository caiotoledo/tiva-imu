#ifndef LOG_H_
#define LOG_H_

#include <utils/uartstdio.h>

/* Enable INFO logs */
#define LOG_DEBUG_INFO
/* Enable WARNING logs */
// #define LOG_DEBUG_WARN
/* Enable ERROR logs */
// #define LOG_DEBUG_ERROR

#define LOG_UART(...)               \
do {                                \
        UARTprintf(__VA_ARGS__);    \
        UARTprintf("\r\n");         \
} while(0)                          \

#define LOG_DEBUG_UART(LEVEL, ...)      \
do {                                    \
        UARTprintf("[%s] ",__func__);   \
        UARTprintf("%s: ",#LEVEL);      \
        UARTprintf(__VA_ARGS__);        \
        UARTprintf("\r\n");             \
} while(0)                              \

#ifdef LOG_DEBUG_INFO
    #define LOG_DEBUG_WARN
    #define INFO(...)       LOG_DEBUG_UART(INFO, __VA_ARGS__)
#else
    #define INFO(...)
#endif

#ifdef LOG_DEBUG_WARN
    #define LOG_DEBUG_ERROR
    #define WARN(...)      LOG_DEBUG_UART(WARN, __VA_ARGS__)
#else
    #define WARN(...)
#endif

#ifdef LOG_DEBUG_ERROR
    #define ERROR(...)      LOG_DEBUG_UART(ERROR, __VA_ARGS__)
#else
    #define ERROR(...)
#endif

#endif /* LOG_H_ */
