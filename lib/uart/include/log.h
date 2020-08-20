#ifndef LOG_H_
#define LOG_H_

#include <utils/uartstdio.h>

#define LOG_UART(...)           \
do {                            \
    UARTprintf(__VA_ARGS__);    \
    UARTprintf("\r\n");         \
} while(0)                      \

#define LOG_DEBUG_UART(LEVEL, ...)  \
do {                                \
    UARTprintf("[%s] ",__func__);   \
    UARTprintf("%s: ",#LEVEL);      \
    UARTprintf(__VA_ARGS__);        \
    UARTprintf("\r\n");             \
} while(0)                          \

/* MACRO for INFO logs */
#ifdef LOG_DEBUG_INFO
    #define LOG_DEBUG_WARN
    #define INFO(...)       LOG_DEBUG_UART(INFO, __VA_ARGS__)
#else
    #define INFO(...)
#endif

/* MACRO for WARNING logs */
#ifdef LOG_DEBUG_WARN
    #define LOG_DEBUG_ERROR
    #define WARN(...)      LOG_DEBUG_UART(WARN, __VA_ARGS__)
#else
    #define WARN(...)
#endif

/* MACRO for ERROR logs */
#ifdef LOG_DEBUG_ERROR
    #define ERROR(...)      LOG_DEBUG_UART(ERROR, __VA_ARGS__)
#else
    #define ERROR(...)
#endif

#endif /* LOG_H_ */
