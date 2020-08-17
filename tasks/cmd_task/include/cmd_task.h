#ifndef CMD_TASK_H_
#define CMD_TASK_H_

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>

/**
 * @brief Command Task parser
 *
 * @param pvParameters Not used, should be NULL
 */
void vCMDTask(void *pvParameters);

/**
 * @brief Register Functions Commands based on FreeRTOS-CLI library
 *
 * @param pxCommandToRegister Function Command parameters
 * @return int Return 0 if successful
 */
int CMD_RegFuncCommand(const CLI_Command_Definition_t *pxCommandToRegister);

#endif /* CMD_TASK_H_ */
