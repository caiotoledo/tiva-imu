#ifndef CMD_TASK_H_
#define CMD_TASK_H_

typedef enum {
  eGetValue,
  eSetValue,
  eInvalidValue
} eCommandType;

typedef struct {
  eCommandType eCmdType;
  float value;
} stCommandParam;

typedef void (*vFuncCommand)(stCommandParam);

/**
 * @brief Command Task parser
 *
 * @param pvParameters Not used, should be NULL
 */
void vCMDTask(void *pvParameters);

/**
 * @brief Register
 *
 * @param cmd String command
 * @param func Function pointer for the command
 * @return int Return 0 when successful
 */
int CMD_RegFunc(const char *cmd, vFuncCommand func);

#endif /* CMD_TASK_H_ */
