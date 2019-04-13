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

/* FreeRTOS Macros Tasks */
#define TASK_LED_PRIORITY       (tskIDLE_PRIORITY)
#define TASK_LED_STACKSIZE      (configMINIMAL_STACK_SIZE)
#define TASK_I2C_PRIORITY       (TASK_LED_PRIORITY+1)
#define TASK_I2C_STACKSIZE      (configMINIMAL_STACK_SIZE)

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
        GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED);
        vTaskDelay(500 / portTICK_RATE_MS);
        GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void vI2CTask (void *pvParam)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1));
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);

    for(;;)
    {
        I2CMasterSlaveAddrSet(I2C1_BASE, 0x68, false);
        I2CMasterDataPut(I2C1_BASE, 0x75);
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy(I2C1_BASE));
        I2CMasterSlaveAddrSet(I2C1_BASE, 0x68, true);
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(I2CMasterBusy(I2C1_BASE));
        uint32_t val = I2CMasterDataGet(I2C1_BASE);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED | LED_BLUE | LED_GREEN);

    if (xTaskCreate(vLedTask, "LED Task", TASK_LED_STACKSIZE, NULL, TASK_LED_PRIORITY, NULL) != pdPASS)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED | LED_GREEN | LED_BLUE);
        while(1);
    }

    if (xTaskCreate(vI2CTask, "I2C Task", TASK_I2C_STACKSIZE, NULL, TASK_I2C_PRIORITY, NULL) != pdPASS)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, LED_RED | LED_GREEN | LED_BLUE);
        while(1);
    }

    vTaskStartScheduler();

    while (1);
}
