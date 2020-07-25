#include <stdint.h>
#include <string.h>

#include <hal_uart.h>
#include <log.h>

#include <FreeRTOS.h>
#include <task.h>

#include "cmd_task.h"
#include "cmd_internal.h"

void vCMDTask(void *pvParameters)
{
  INFO("Start [%s]", __func__);

  char buf[128] = { 0 };
  for (;;)
  {
    size_t size = sizeof(buf);
    int ret = UART_GetLine(buf, &size);

    if (ret == 0)
    {
      stCommandParam param;
      vFuncCommand func;
      int retParser = parseCmd(buf, &func, &param);

      if (retParser == 0)
      {
        INFO("Execute Command [%s]", buf);
        func(param);
      }
      else
      {
        WARN("Command not found [%s]", buf);
      }
    }
  }

  ERROR("Finish [%s]", __func__);
  /* This task can be suspended now */
  vTaskSuspend(NULL);
}

int CMD_RegFunc(const char *cmd, vFuncCommand func)
{
  return regFunc(cmd, func);
}
