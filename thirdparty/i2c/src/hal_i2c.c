#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include <tm4c123gh6pm_target_definitions.h>

#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include <hal_i2c.h>

#define I2C_TIMEOUT_MS      200

typedef struct
{
    uint32_t I2CPeripheral;
    uint32_t I2CBase;
    uint32_t GPIOPeripheral;
    uint32_t GPIOPortBase;
    uint32_t GPIOPinConfSCL;
    uint32_t GPIOPinConfSDA;
    uint32_t GPIOPinSCL;
    uint32_t GPIOPinSDA;
} tI2C_Pin_Conf;

typedef struct
{
    eI2C_BASE i2c;
    tI2C_Pin_Conf conf;
} tI2C_Conf;

static const tI2C_Pin_Conf *GetI2CConf(eI2C_BASE i2c);

/* Map for I2Cs configurations: */
static const tI2C_Conf i2c_configurations[] =
    {
        {
            .i2c = I2C1,
            .conf =
                {
                    .I2CPeripheral =    SYSCTL_PERIPH_I2C1,
                    .I2CBase =          I2C1_BASE,
                    .GPIOPeripheral =   SYSCTL_PERIPH_GPIOA,
                    .GPIOPortBase =     GPIO_PORTA_BASE,
                    .GPIOPinConfSCL =   GPIO_PA6_I2C1SCL,
                    .GPIOPinConfSDA =   GPIO_PA7_I2C1SDA,
                    .GPIOPinSCL =       GPIO_PIN_6,
                    .GPIOPinSDA =       GPIO_PIN_7,
                },
        },
};
typedef uint32_t (*millis)();
static millis ms_func;

static const tI2C_Pin_Conf *GetI2CConf(eI2C_BASE i2c)
{
    const tI2C_Pin_Conf *i2c_pin_conf = NULL;
    /* Search for I2C configuration */
    for (uint8_t i = 0; i < sizeof(i2c_configurations) / sizeof(tI2C_Conf); i++)
    {
        if (i2c_configurations[i].i2c == i2c)
        {
            i2c_pin_conf = &i2c_configurations[i].conf;
            break;
        }
    }
    return i2c_pin_conf;
}

static bool WaitI2CMasterBusy(eI2C_BASE i2c, uint32_t timeout)
{
    bool flag = false;

    uint32_t start = ms_func();

    const tI2C_Pin_Conf *i2c_pin_conf;
    i2c_pin_conf = GetI2CConf(i2c);
    /* Return if no valid configuration was found */
    if (i2c_pin_conf == NULL)
    {
        goto end_wait;
    }

    while (I2CMasterBusy(i2c_pin_conf->I2CBase))
    {
        if ( (ms_func() - start) > timeout )
        {
            goto end_wait;
        }
    }

    /* Only reaches this point if sucessful */
    flag = true;
end_wait:
    return flag;
}

void I2C_Enable(eI2C_BASE i2c, millis func)
{

    /* Store milliseconds function: */
    if (func != 0U)
    {
        ms_func = func;
    }

    const tI2C_Pin_Conf *i2c_pin_conf;
    i2c_pin_conf = GetI2CConf(i2c);
    /* Return if no valid configuration was found */
    if (i2c_pin_conf == NULL)
    {
        return;
    }

    /* Enable Peripheral's Clock */
    SysCtlPeripheralEnable(i2c_pin_conf->GPIOPeripheral);
    SysCtlPeripheralEnable(i2c_pin_conf->I2CPeripheral);

    /* Configure its GPIOs */
    GPIOPinConfigure(i2c_pin_conf->GPIOPinConfSCL);
    GPIOPinConfigure(i2c_pin_conf->GPIOPinConfSDA);
    GPIOPinTypeI2CSCL(i2c_pin_conf->GPIOPortBase, i2c_pin_conf->GPIOPinSCL);
    GPIOPinTypeI2C(i2c_pin_conf->GPIOPortBase, i2c_pin_conf->GPIOPinSDA);

    /* Set fast speed 400KHz */
    I2CMasterInitExpClk(i2c_pin_conf->I2CBase, SysCtlClockGet(), true);
}

uint32_t I2C_Read_Reg(eI2C_BASE i2c, uint8_t slave_addr, uint8_t reg_addr)
{
    uint32_t val = 0x00;

    const tI2C_Pin_Conf *i2c_pin_conf;
    i2c_pin_conf = GetI2CConf(i2c);
    /* Return if no valid configuration was found */
    if (i2c_pin_conf == NULL)
    {
        goto end_read;
    }

    /* Set slave address */
    I2CMasterSlaveAddrSet(i2c_pin_conf->I2CBase, slave_addr, false);

    /* Set register address */
    I2CMasterDataPut(i2c_pin_conf->I2CBase, reg_addr);
    I2CMasterControl(i2c_pin_conf->I2CBase, I2C_MASTER_CMD_BURST_SEND_START);

    if (!WaitI2CMasterBusy(i2c, I2C_TIMEOUT_MS))
    {
        goto end_read;
    }

    /* Get register value */
    I2CMasterSlaveAddrSet(i2c_pin_conf->I2CBase, slave_addr, true);
    I2CMasterControl(i2c_pin_conf->I2CBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

    if (!WaitI2CMasterBusy(i2c, I2C_TIMEOUT_MS))
    {
        goto end_read;
    }

    /* Return register value */
    val = I2CMasterDataGet(i2c_pin_conf->I2CBase);

end_read:
    return val;
}

void I2C_Send(eI2C_BASE i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t num_of_args, ...)
{
    const tI2C_Pin_Conf *i2c_pin_conf;
    i2c_pin_conf = GetI2CConf(i2c);
    /* Return if no valid configuration was found */
    if (i2c_pin_conf == NULL)
    {
        return;
    }

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(i2c_pin_conf->I2CBase, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(i2c_pin_conf->I2CBase, reg_addr);

    //Initiate send of data from the MCU
    I2CMasterControl(i2c_pin_conf->I2CBase, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until MCU is done transferring.
    if (!WaitI2CMasterBusy(i2c, I2C_TIMEOUT_MS))
    {
        return;
    }

    //send num_of_args-2 pieces of data, using the
    //BURST_SEND_CONT command of the I2C module
    for (uint8_t i = 0; i < num_of_args; i++)
    {
        //put next piece of data into I2C FIFO
        I2CMasterDataPut(i2c_pin_conf->I2CBase, va_arg(vargs, uint32_t));

        if (i < (num_of_args-1) )
        {
            /* Send Cont Burst command */
            I2CMasterControl(i2c_pin_conf->I2CBase, I2C_MASTER_CMD_BURST_SEND_CONT);
        }
        else
        {
            /* Send Last Burst command */
            I2CMasterControl(i2c_pin_conf->I2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
        }

        // Wait until MCU is done transferring.
        if (!WaitI2CMasterBusy(i2c, I2C_TIMEOUT_MS))
        {
            return;
        }
    }

    //"close" variable args list
    va_end(vargs);
}
