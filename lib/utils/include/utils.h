#ifndef UTILS_H_
#define UTILS_H_

#include <stddef.h>
#include <stdint.h>

#define ABS(x)    (x < 0 ? (-x) : (x))
#define MIN(a,b)  ((a > b) ? b : a)
#define MAX(a,b)  ((a > b) ? a : b)

#define UNUSED(x) ((void)x)

/**
 * @brief Convert a String to a Number
 *
 * @param buf String buffer with NULL Terminator
 * @param len String Length
 * @return int64_t Return Number extracted
 */
int64_t UTILS_ConvertStringToNumber(const char *buf, size_t len);

#endif /* UTILS_H_ */
