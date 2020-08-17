#include <stdint.h>
#include <string.h>

#include <hal_uart.h>
#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOS_CLI.h>

#include "cmd_task.h"

void vCMDTask(void *pvParameters)
{
  INFO("Start [%s]", __func__);

  char outputBuffer[128] = { 0 };
  char inputBuffer[128] = { 0 };
  for (;;)
  {
    UARTprintf(">> "); /* Delimiter to show that the command line is available */
    size_t size = sizeof(inputBuffer);
    int ret = UART_GetLine(inputBuffer, &size, true);
    UARTprintf("\n"); /* Add a new linefeed at the end of each command echoed */

    if ((ret == 0) && (size > 0))
    {
      BaseType_t xReturned;
      do
      {
        /* Get the next output string from the command interpreter. */
        xReturned = FreeRTOS_CLIProcessCommand( inputBuffer, outputBuffer, sizeof(outputBuffer) );

        /* Write the generated string to the UART. */
        UARTprintf(outputBuffer);

      } while( xReturned != pdFALSE );
    }
  }

  ERROR("Finish [%s]", __func__);
  /* This task can be suspended now */
  vTaskSuspend(NULL);
}

int CMD_RegFuncCommand(const CLI_Command_Definition_t *pxCommandToRegister)
{
  int ret = 0;

  if (FreeRTOS_CLIRegisterCommand( pxCommandToRegister ) != pdPASS)
  {
    ERROR("Not enough Heap Memory!");
    ret = -1;
  }

  return ret;
}
