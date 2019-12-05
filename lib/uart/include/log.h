#ifndef LOG_H_
#define LOG_H_

#include <utils/uartstdio.h>

/* Enable INFO logs */
#define LOG_DEBUG_INFO
/* Enable WARNING logs */
// #define LOG_DEBUG_WARN
/* Enable ERROR logs */
// #define LOG_DEBUG_ERROR

#ifdef LOG_DEBUG_INFO
    #define LOG_DEBUG_WARN
    #define INFO(...)      UARTprintf("[%s] INFO: ",__func__); UARTprintf(__VA_ARGS__); UARTprintf("\r\n");
#else
    #define INFO(...)
#endif

#ifdef LOG_DEBUG_WARN
    #define LOG_DEBUG_ERROR
    #define WARN(...)      UARTprintf("[%s] WARN: ",__func__); UARTprintf(__VA_ARGS__); UARTprintf("\r\n");
#else
    #define WARN(...)
#endif

#ifdef LOG_DEBUG_ERROR
    #define ERROR(...)      UARTprintf("[%s] ERROR: ",__func__); UARTprintf(__VA_ARGS__); UARTprintf("\r\n");
#else
    #define ERROR(...)
#endif

#endif /* LOG_H_ */
