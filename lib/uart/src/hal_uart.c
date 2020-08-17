#include <stdint.h>
#include <stdbool.h>

#include <tm4c123gh6pm_target_definitions.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#include <hal_uart.h>

typedef struct
{
    uint32_t UARTPeripheral;
    uint32_t UARTBase;
    uint32_t UARTClockBase;
    uint32_t GPIOPeripheral;
    uint32_t GPIOPortBase;
    uint32_t GPIOPinConfigTX;
    uint32_t GPIOPinTX;
    uint32_t GPIOPinConfigRX;
    uint32_t GPIOPinRX;
} tUART_Pin_Conf;

typedef struct
{
    eUART_BASE uart;
    tUART_Pin_Conf conf;
} tUART_Conf;

static const tUART_Conf uart_configurations[] =
{
    {
        .uart = UART0,
        .conf =
        {
            .UARTPeripheral = SYSCTL_PERIPH_UART0,
            .UARTBase = UART0_BASE,
            .UARTClockBase = UART_CLOCK_PIOSC,
            .GPIOPeripheral = SYSCTL_PERIPH_GPIOA,
            .GPIOPortBase = GPIO_PORTA_BASE,
            .GPIOPinConfigTX = GPIO_PA1_U0TX,
            .GPIOPinTX = GPIO_PIN_1,
            .GPIOPinConfigRX = GPIO_PA0_U0RX,
            .GPIOPinRX = GPIO_PIN_0,
        }
    }
};

static const tUART_Pin_Conf *GetUARTConf(eUART_BASE uart);

void UART_Enable(eUART_BASE uart, uint32_t baudrate)
{
    const tUART_Pin_Conf *conf = GetUARTConf(uart);
    /* Return if no valid configuration was found */
    if (conf == 0U)
    {
        return;
    }

    /* Enable the GPIO Peripheral used by the UART. */
    SysCtlPeripheralEnable(conf->GPIOPeripheral);

    /* Enable UART0 */
    SysCtlPeripheralEnable(conf->UARTPeripheral);

    /* Configure GPIO Pins for UART mode. */
    GPIOPinConfigure(conf->GPIOPinConfigRX);
    GPIOPinConfigure(conf->GPIOPinConfigTX);
    GPIOPinTypeUART(conf->GPIOPortBase, conf->GPIOPinRX | conf->GPIOPinTX);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(conf->UARTBase, conf->UARTClockBase);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, baudrate, 16000000);
}

int UART_GetLine(char *str, size_t *size, bool echo)
{
    /* Verify pointers */
    if ((str == NULL) || (size == NULL))
    {
        return -1;
    }

    int ret = 0;
    bool hasSpace = true;
    size_t index = 0;
    while (true)
    {
        /* Get a single char */
        char c = UARTgetc();
        if (echo)
        {
            /* Echo each char received */
            UARTwrite(&c, 1);
        }

        /* Stop when received new line and carrier return */
        if ((c == '\n') || (c == '\r'))
        {
            str[index++] = '\0'; /* Add a NULL terminator at the end of the string */
            break;
        }

        if (hasSpace)
        {
            /* Store new char and add a NULL terminator */
            str[index++] = c;

            /* Check if there is enough size */
            if (index == ((*size)-1))
            {
                hasSpace = false;
            }
        }
        else
        {
            ret = -1;
        }
    };

    /* Store size */
    (*size) = index;

    return ret;
}

static const tUART_Pin_Conf *GetUARTConf(eUART_BASE uart)
{
    const tUART_Pin_Conf *uart_pin_conf = 0U;
    /* Search for UART configuration */
    for (uint8_t i = 0; i < sizeof(uart_configurations) / sizeof(tUART_Conf); i++)
    {
        if (uart_configurations[i].uart == uart)
        {
            uart_pin_conf = &uart_configurations[i].conf;
            break;
        }
    }
    return uart_pin_conf;
}
