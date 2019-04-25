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
#include <LED_RGB.h>
#include <hal_i2c.h>
#include <hal_uart.h>
#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#include "led_task.h"

/* FreeRTOS Macros Tasks */
#define TASK_LED_PRIORITY       (tskIDLE_PRIORITY)
#define TASK_LED_STACKSIZE      (configMINIMAL_STACK_SIZE)
#define TASK_I2C_PRIORITY       (TASK_LED_PRIORITY+1)
#define TASK_I2C_STACKSIZE      (configMINIMAL_STACK_SIZE)

#define UNUSED(x)               ((void)x)

uint32_t GetMillis(void);

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

void vI2CTask (void *pvParam)
{
    I2C_Enable(I2C1);
    for(;;)
    {
        uint32_t val = I2C_Read_Reg(I2C1, 0x68, 0x75);
        if (val == 0x68)
        {
            INFO("MPU6050 found [0x%02X]\n", val);
        }
        else
        {
            ERROR("MPU6050 NOT found [0x%02X]\n", val);
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

uint32_t GetMillis(void)
{
    return (xTaskGetTickCountFromISR()/portTICK_RATE_MS);
}

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    /* Initialize UART */
    UART_Enable(UART0, 115200);

    /* Start Program */
    INFO("Start Program\n");
    INFO("Version: %u.%u.%u.%d\n", MAJOR_VERSION, MINOR_VERSION, RELEASE_VERSION, __INT_TIMESTAMP__);

    /* Initialize LEDs */
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
