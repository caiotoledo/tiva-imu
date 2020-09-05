#ifndef UTILS_H_
#define UTILS_H_

#include <stddef.h>
#include <stdint.h>

#define MIN(a,b)  ((a > b) ? b : a)
#define MAX(a,b)  ((a > b) ? a : b)

/**
 * @brief Convert a String to a Number
 *
 * @param buf String buffer with NULL Terminator
 * @param len String Length
 * @return int64_t Return Number extracted
 */
int64_t UTILS_ConvertStringToNumber(const char *buf, size_t len);

#endif /* UTILS_H_ */
