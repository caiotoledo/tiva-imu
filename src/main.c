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

#include <LED_RGB.h>
#include <hal_i2c.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

/* FreeRTOS Macros Tasks */
#define TASK_LED_PRIORITY       (tskIDLE_PRIORITY)
#define TASK_LED_STACKSIZE      (configMINIMAL_STACK_SIZE)
#define TASK_I2C_PRIORITY       (TASK_LED_PRIORITY+1)
#define TASK_I2C_STACKSIZE      (configMINIMAL_STACK_SIZE)

#define UNUSED(x)               ((void)x)

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void)pcTaskName;
    (void)xTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for (;;)
    {
    }
}

void vLedTask(void *pvParameters)
{
    for (;;)
    {
        LED_Set(RED_LED, true);
        vTaskDelay(500 / portTICK_RATE_MS);
        LED_Set(RED_LED, false);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void vI2CTask (void *pvParam)
{
    I2C_Enable(I2C1);
    for(;;)
    {
        uint32_t val = I2C_Read_Reg(I2C1, 0x68, 0x75);
        UNUSED(val);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    LED_Enable();

    if (xTaskCreate(vLedTask, "LED Task", TASK_LED_STACKSIZE, NULL, TASK_LED_PRIORITY, NULL) != pdPASS)
    {
        LED_Set(RED_LED, true);
        while(1);
    }

    if (xTaskCreate(vI2CTask, "I2C Task", TASK_I2C_STACKSIZE, NULL, TASK_I2C_PRIORITY, NULL) != pdPASS)
    {
        LED_Set(RED_LED, true);
        while(1);
    }

    vTaskStartScheduler();

    while (1);
}
