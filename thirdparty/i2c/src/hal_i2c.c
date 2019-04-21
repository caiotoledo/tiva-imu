#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

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

static const tI2C_Pin_Conf *GetI2CConf(eI2C_BASE i2c)
{
    const tI2C_Pin_Conf *i2c_pin_conf = 0U;
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

void I2C_Enable(eI2C_BASE i2c)
{
    const tI2C_Pin_Conf *i2c_pin_conf;

    i2c_pin_conf = GetI2CConf(i2c);
    /* Return if no valid configuration was found */
    if (i2c_pin_conf == 0U)
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
    /* Set slave address */
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);

    /* Set register address */
    I2CMasterDataPut(I2C1_BASE, reg_addr);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C1_BASE));

    /* Get register value */
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C1_BASE));

    /* Return register value */
    return I2CMasterDataGet(I2C1_BASE);
}