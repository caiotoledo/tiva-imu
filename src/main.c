#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "tm4c123gh6pm_target_definitions.h"

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include <version.h>
#include <rgb.h>
#include <hal_uart.h>
#include <mcu.h>
#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#include <tasks_config.h>
#include <led_task.h>
#include <cmd_task.h>
#include <imu_task.h>

#define PROGRAM_NAME            "tiva-imu"

#define UNUSED(x)               ((void)x)

static BaseType_t VersionProgramCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static const CLI_Command_Definition_t xVersionProgram =
{
    "ver", /* The command string. */
    "ver:\r\n Show program version\r\n", /* Help string. */
    VersionProgramCommand, /* The function to run. */
    0 /* No parameters are expected. */
};

static BaseType_t VersionProgramCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    UNUSED(pcCommandString);

    LOG_UART("Version: %u.%u.%u-%d git@%s%s", MAJOR_VERSION, MINOR_VERSION, RELEASE_VERSION, __INT_TIMESTAMP__, COMMIT_HASH, LOCAL_DIRTY);

    /* Do not use the pcWriteBuffer to output on console */
    if (xWriteBufferLen >= 1)
    {
        pcWriteBuffer[0] = '\0';
    }

    return pdFALSE;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    UNUSED(pcTaskName);
    UNUSED(xTask);

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for (;;)
    {
    }
}

static void vFaultFunc(void)
{
    LED_Set(RED_LED, true);
    while (1);
}

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    /* Initialize UART */
    UART_Enable(UART0, 115200);

    /* Start Program */
    LOG_UART("Start %s", PROGRAM_NAME);
    LOG_UART("Version: %u.%u.%u-%d git@%s%s\n", MAJOR_VERSION, MINOR_VERSION, RELEASE_VERSION, __INT_TIMESTAMP__, COMMIT_HASH, LOCAL_DIRTY);

    /* Register Command Functions */
    CMD_RegFuncCommand(&xVersionProgram);

    /* Initialize LEDs */
    LED_Enable();

    if (xTaskCreate(vLedTask, "LED Task", TASK_LED_STACKSIZE, NULL, TASK_LED_PRIORITY, NULL) != pdPASS)
    {
        vFaultFunc();
    }

    if (xTaskCreate(vCMDTask, "CMD Task", TASK_CMD_STACKSIZE, NULL, TASK_CMD_PRIORITY, NULL) != pdPASS)
    {
        vFaultFunc();
    }

    if (xTaskCreate(vIMUTask, "IMU Task", TASK_IMU_STACKSIZE, NULL, TASK_IMU_PRIORITY, NULL) != pdPASS)
    {
        vFaultFunc();
    }

    /* Should never return */
    vTaskStartScheduler();

    /* Reset the system if the scheduler returns */
    MCU_SoftwareReset();

    while (1);
}
