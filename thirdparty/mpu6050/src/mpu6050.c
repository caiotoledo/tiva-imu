#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <log.h>
#include <hal_i2c.h>
#include <mpu6050.h>

#define INVALID_ADDR       (0xFF)

typedef enum
{
    MPU6050_SMPLRT_DIV = 0x19,

    MPU6050_CONFIG = 0x1A,
    MPU6050_GYRO_CONFIG,
    MPU6050_ACCEL_CONFIG,

    MPU6050_INT_ENABLE = 0x38,

    MPU6050_ACCEL_XOUT_H = 0x3B,
    MPU6050_ACCEL_XOUT_L,
    MPU6050_ACCEL_YOUT_H,
    MPU6050_ACCEL_YOUT_L,
    MPU6050_ACCEL_ZOUT_H,
    MPU6050_ACCEL_ZOUT_L,
    MPU6050_TEMP_OUT_H,
    MPU6050_TEMP_OUT_L,
    MPU6050_GYRO_XOUT_H,
    MPU6050_GYRO_XOUT_L,
    MPU6050_GYRO_YOUT_H,
    MPU6050_GYRO_YOUT_L,
    MPU6050_GYRO_ZOUT_H,
    MPU6050_GYRO_ZOUT_L,

    MPU6050_USER_CTRL = 0x6A,
    MPU6050_PWR_MGMT_1,
    MPU6050_PWR_MGMT_2,

    MPU6050_WHO_AM_I = 0x75
} tMPU6050_Addr_Reg;

typedef struct
{
    tMPU6050_Addr_Reg   AddrReg;
    uint32_t            val;
} tMPU6050_Addr_Map;

typedef struct
{
    eI2C_BASE i2c;
    tMPU6050_Addr_Map *map;
} tMPU6050_Data;

typedef struct
{
    const eMPU6050_BASE mpu;
    const uint32_t addr;
    tMPU6050_Data conf;
} tMPU6050_Conf;

static tMPU6050_Conf *GetMPU6050Conf(eMPU6050_BASE mpu);
/* TODO: Implement function: */
// static void SetMPU6050Data(eMPU6050_BASE mpu, tMPU6050_Data *data);

static tMPU6050_Addr_Map reg_configuration[] =
{
    { .AddrReg = MPU6050_SMPLRT_DIV,    .val = 0x00, },
    { .AddrReg = MPU6050_CONFIG,        .val = 0x00, },
    { .AddrReg = MPU6050_GYRO_CONFIG,   .val = 0x00, },
    { .AddrReg = MPU6050_ACCEL_CONFIG,  .val = 0x00, },
    { .AddrReg = MPU6050_INT_ENABLE,    .val = 0x00, },
    { .AddrReg = MPU6050_USER_CTRL,     .val = 0x00, },
    { .AddrReg = MPU6050_PWR_MGMT_1,    .val = 0x00, },
    { .AddrReg = INVALID_ADDR,          .val = 0xFF, },
};

/* Map for MPUs configurations: */
static tMPU6050_Conf mpu_configurations[] =
{
    {
        .mpu = MPU6050_LOW,
        .addr = 0x68,
        .conf =
            {
                .i2c =  I2C0,
                .map = reg_configuration,
            },
    },
    {
        .mpu = MPU6050_HIGH,
        .addr = 0x69,
        .conf =
            {
                .i2c =  I2C0,
                .map = reg_configuration,
            },
    },
};

typedef uint32_t (*millis)();
static millis ms_func;

int MPU6050_Enable(eMPU6050_BASE mpu, eI2C_BASE i2c, uint32_t (*func)())
{
    int ret = -1;

    /* Store milliseconds function: */
    if (func != 0U)
    {
        ms_func = func;
    }

    tMPU6050_Conf *mpu_conf;
    mpu_conf = GetMPU6050Conf(mpu);
    /* Return if no valid configuration was found */
    if (mpu_conf == NULL)
    {
        ERROR("Invalid Configuration!");
        goto end_mpu6050_enable;
    }

    /* Enable I2C interface: */
    int retI2C = I2C_Enable(i2c, func);
    if (retI2C != 0)
    {
        ERROR("Failed to enable I2C Interface!");
        goto end_mpu6050_enable;
    }

    /* Verify the IMU presence */
    uint32_t who_am_I = I2C_Read_Reg(I2C1, mpu_conf->addr, MPU6050_WHO_AM_I);
    if (who_am_I != mpu_conf->addr)
    {
        ERROR("Device not found [0x%04X]", who_am_I);
        goto end_mpu6050_enable;
    }

    /* Update MPU6050 I2C Interface */
    mpu_conf->conf.i2c = i2c;

    tMPU6050_Addr_Map *registers_map = mpu_conf->conf.map;
    /* Configure MPU6050 */
    for (size_t i = 0; registers_map[i].AddrReg != INVALID_ADDR; i++)
    {
        I2C_Send(mpu_conf->conf.i2c, mpu_conf->addr, registers_map[i].AddrReg, 1, registers_map[i].val);
    }

    ret = 0;
end_mpu6050_enable:
    return ret;
}

static tMPU6050_Conf *GetMPU6050Conf(eMPU6050_BASE mpu)
{
    tMPU6050_Conf *mpu_conf = NULL;
    /* Search for I2C configuration */
    for (uint8_t i = 0; i < sizeof(mpu_configurations) / sizeof(tMPU6050_Conf); i++)
    {
        if (mpu_configurations[i].mpu == mpu)
        {
            mpu_conf = &mpu_configurations[i];
            break;
        }
    }
    return mpu_conf;
}
