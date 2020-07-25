#include <string.h>

#include "cmd_internal.h"

#define STR_CMD_SIZE    (16U)
#define FUNC_CMD_SIZE   (10U)

typedef struct {
  char str[STR_CMD_SIZE];
  vFuncCommand func;
  stCommandParam param;
} cmd2Func_t;

static vFuncCommand Cmd2Func(char *str);

static cmd2Func_t mapFunc[FUNC_CMD_SIZE] = { 0 };
static size_t lenMapFunc = 0;

int regFunc(const char *cmd, vFuncCommand func)
{
  if (!func)
  {
    return -1;
  }

  if (!cmd)
  {
    return -1;
  }

  if (lenMapFunc >= (FUNC_CMD_SIZE-1))
  {
    return -1;
  }

  /* Check if the command already exists */
  for (size_t i = 0; i < lenMapFunc; ++i)
  {
    if ( strcmp(cmd, mapFunc[i].str) == 0 )
    {
      /* Command found! */
      mapFunc[i].func = func;
      return 0;
    }
  }

  /* Adding new Function to map: */
  mapFunc[lenMapFunc].func = func;
  strncpy(mapFunc[lenMapFunc].str, cmd, sizeof(mapFunc[lenMapFunc].str));
  lenMapFunc++;
  return 0;
}

int parseCmd(char *str, vFuncCommand *func, stCommandParam *param)
{
  /* Verify null pointer */
  if (func == NULL || param == NULL)
  {
    return -1;
  }

  param->value = 0;
  param->eCmdType = eInvalidValue;

  if (strstr(str,";"))
  {
    /* TODO: Implement complex parser */
  }
  else
  {
    *func = Cmd2Func(str);
    if ((*func) == NULL)
    {
      return -1;
    }
  }

  return 0;
}

static vFuncCommand Cmd2Func(char *str)
{
  for (size_t i = 0; i < lenMapFunc; ++i)
  {
    if ( strcmp(str, mapFunc[i].str) == 0 )
    {
      /* Command found! */
      return mapFunc[i].func;
    }
  }

  /* If command isn't found return null */
  return NULL;
}
