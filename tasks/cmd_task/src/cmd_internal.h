#ifndef CMD_INTERNAL_H_
#define CMD_INTERNAL_H_

#include "cmd_task.h"

/**
 * @brief Internal function to register a vFuncCommand function
 *
 * @param cmd String command
 * @param func Function pointer for the command
 * @return int Return 0 when successful
 */
int regFunc(const char *cmd, vFuncCommand func);

/**
 * @brief Parse a string command and its parameters
 *
 * @param str String command
 * @param func Function pointer to execute the command
 * @param param Parameters parsed for the command
 * @return int Return 0 when successful
 */
int parseCmd(char *str, vFuncCommand *func, stCommandParam *param);

#endif /* CMD_INTERNAL_H_ */
