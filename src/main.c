#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3

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
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED);
        vTaskDelay(500 / portTICK_RATE_MS);
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void vI2CTask (void *pvParam)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1));
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), true);

    for(;;)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

int main()
{
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED | LED_BLUE | LED_GREEN);

    if (xTaskCreate(vLedTask, "LED Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL) != pdPASS)
    {
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED | LED_GREEN | LED_BLUE);
        while(1);
    }

    if (xTaskCreate(vI2CTask, "I2C Task", configMINIMAL_STACK_SIZE, NULL, 1+tskIDLE_PRIORITY, NULL) != pdPASS)
    {
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED | LED_GREEN | LED_BLUE);
        while(1);
    }

    vTaskStartScheduler();

    while (1);
}
