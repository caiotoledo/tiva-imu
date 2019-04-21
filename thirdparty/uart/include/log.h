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
    #define INFO(...)      UARTprintf(__VA_ARGS__)
#else
    #define INFO(...)
#endif

#ifdef LOG_DEBUG_WARN
    #define LOG_DEBUG_ERROR
    #define WARN(...)      UARTprintf(__VA_ARGS__)
#else
    #define WARN(...)
#endif

#ifdef LOG_DEBUG_ERROR
    #define ERROR(...)      UARTprintf(__VA_ARGS__)
#else
    #define ERROR(...)
#endif

#endif /* LOG_H_ */
