#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <log.h>
#include <mpu6050.h>

#define INVALID_ADDR                (0xFF)

#define CONST_ACCEL                 (16.384)
#define CONST_GYRO                  (131)
#define CONST_TEMP                  (340)
#define OFFSET_TEMP                 (36.53F)

#define DEFAULT_I2C_INTERFACE       (I2C0)

#define GET_CONF(pConf, mpu, error_goto)            \
do {                                                \
    pConf = GetMPU6050Conf(mpu);                    \
    if (pConf == NULL)                              \
    {                                               \
        ERROR("Invalid Configuration! [%d]", mpu);  \
        goto error_goto;                            \
    }                                               \
} while(0)                                          \

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
    uint8_t             val;
} tMPU6050_Addr_Map;

typedef struct
{
    eI2C_BASE i2c;
    tMPU6050_Addr_Map *map;
    accel_t accelOffset;
    gyro_t gyroOffset;
} tMPU6050_Data;

typedef struct
{
    const eMPU6050_BASE mpu;
    const uint32_t addr;
    tMPU6050_Data conf;
} tMPU6050_Conf;

static double ConvertIMUVal(uint16_t val, double constant);
inline static tMPU6050_Conf *GetMPU6050Conf(eMPU6050_BASE mpu);
static void ApplyAccelOffset(accel_t offset, accel_t *accel);
static void ApplyGyroOffset(gyro_t offset, gyro_t *gyro);
/* TODO: Implement function: */
// static void SetMPU6050Data(eMPU6050_BASE mpu, tMPU6050_Data *data);

static tMPU6050_Addr_Map reg_configuration[] =
{
    { .AddrReg = MPU6050_PWR_MGMT_1,    .val = 0x80, }, /* Reset IMU Registers */
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
                .i2c =  DEFAULT_I2C_INTERFACE,
                .map = reg_configuration,
                .accelOffset = { 0, 0, 0 },
                .gyroOffset = { 0, 0, 0 },
            },
    },
    {
        .mpu = MPU6050_HIGH,
        .addr = 0x69,
        .conf =
            {
                .i2c =  DEFAULT_I2C_INTERFACE,
                .map = reg_configuration,
                .accelOffset = { 0, 0, 0 },
                .gyroOffset = { 0, 0, 0 },
            },
    },
};

typedef uint32_t (*millis)();
static millis ms_func;

int MPU6050_Enable(eMPU6050_BASE mpu, eI2C_BASE i2c, uint32_t (*func)())
{
    int ret = -1;

    /* Check valid function pointer */
    if (func == NULL)
    {
        goto end_mpu6050_enable;
    }
    /* Store milliseconds function: */
    ms_func = func;

    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_enable);

    /* Enable I2C interface: */
    int retI2C = I2C_Enable(i2c, func);
    if (retI2C != 0)
    {
        ERROR("Failed to enable I2C Interface!");
        goto end_mpu6050_enable;
    }

    /* Update MPU6050 I2C Interface */
    eI2C_BASE last_i2c_interface = mpu_conf->conf.i2c;
    mpu_conf->conf.i2c = i2c;

    /* Probe IMU */
    int probe_ret = MPU6050_Probe(mpu);
    if (probe_ret != 0)
    {
        /* Reset its I2C interface if not found */
        mpu_conf->conf.i2c = last_i2c_interface;
        goto end_mpu6050_enable;
    }

    tMPU6050_Addr_Map *registers_map = mpu_conf->conf.map;
    /* Configure MPU6050 */
    for (size_t i = 0; registers_map[i].AddrReg != INVALID_ADDR; i++)
    {
        I2C_Send(mpu_conf->conf.i2c, mpu_conf->addr, registers_map[i].AddrReg, 1, registers_map[i].val);

        /* Power management requires a delay */
        if (registers_map[i].AddrReg == MPU6050_PWR_MGMT_1)
        {
            /* Wait 100 ms for IMU initialization */
            uint32_t start = ms_func();
            while ( (ms_func() - start) <= 100 );
        }
    }

    ret = 0;
end_mpu6050_enable:
    return ret;
}

int MPU6050_Probe(eMPU6050_BASE mpu)
{
    int ret = -1;

    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_probe);

    /* Verify the IMU presence */
    uint32_t who_am_I = I2C_Read_Reg(mpu_conf->conf.i2c, mpu_conf->addr, MPU6050_WHO_AM_I);
    if (who_am_I != mpu_conf->addr)
    {
        ERROR("Device not found [0x%04X]", who_am_I);
        goto end_mpu6050_probe;
    }

    /* Found IMU in I2C bus */
    ret = 0;

end_mpu6050_probe:
    return ret;
}

int MPU6050_ReadAllAccel(eMPU6050_BASE mpu,  accel_t *accel)
{
    int ret = -1;

    /* Pointer protection */
    if (accel == NULL)
    {
        goto end_mpu6050_readaccel;
    }

    /* Get MPU configuration */
    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_readaccel);

    /* Get Raw Acceleration values */
    uint8_t val[6] = { 0 };
    int ret_read = I2C_Read_Multiple_Reg(mpu_conf->conf.i2c, mpu_conf->addr, MPU6050_ACCEL_XOUT_H, sizeof(val), val);
    if (ret_read != sizeof(val))
    {
        ERROR("Read accel registers failed");
        goto end_mpu6050_readaccel;
    }

    /* Acceleration Conversion */
    double output_accel[3] = { 0 };
    for (size_t i = 0; i < sizeof(val); i+=2)
    {
        uint16_t raw_accel = (uint16_t)( (val[i] << 8) | val[i+1] );
        output_accel[i/2] = ConvertIMUVal(raw_accel, CONST_ACCEL);
    }

    /* Store in the output buffer */
    accel->x = output_accel[0];
    accel->y = output_accel[1];
    accel->z = output_accel[2];

    /* Apply Acceleration Offset */
    ApplyAccelOffset(mpu_conf->conf.accelOffset, accel);

    ret = 0;

end_mpu6050_readaccel:
    return ret;
}

int MPU6050_ReadAllGyro(eMPU6050_BASE mpu, gyro_t *gyro)
{
    int ret = -1;

    /* Pointer protection */
    if (gyro == NULL)
    {
        goto end_mpu6050_readgyro;
    }

    /* Get MPU configuration */
    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_readgyro);

    /* Get Raw Gyroscope values */
    uint8_t val[6] = {0};
    int ret_read = I2C_Read_Multiple_Reg(mpu_conf->conf.i2c, mpu_conf->addr, MPU6050_GYRO_XOUT_H, sizeof(val), val);
    if (ret_read != sizeof(val))
    {
        ERROR("Read gyro registers failed");
        goto end_mpu6050_readgyro;
    }

    /* Gyroscope Conversion */
    double output_gyro[3] = {0};
    for (size_t i = 0; i < sizeof(val); i += 2)
    {
        uint16_t raw_gyro = (uint16_t)((val[i] << 8) | val[i + 1]);
        output_gyro[i/2] = ConvertIMUVal(raw_gyro, CONST_GYRO);
    }

    /* Store in the output buffer */
    gyro->x = output_gyro[0];
    gyro->y = output_gyro[1];
    gyro->z = output_gyro[2];

    /* Apply Gyro Offset */
    ApplyGyroOffset(mpu_conf->conf.gyroOffset, gyro);

    ret = 0;

end_mpu6050_readgyro:
    return ret;
}

int MPU6050_ReadTemperature(eMPU6050_BASE mpu, double *temperature)
{
    int ret = -1;

    /* Pointer protection */
    if (temperature == NULL)
    {
        goto end_mpu6050_readtemp;
    }

    /* Get MPU configuration */
    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_readtemp);

    /* Get Raw Temperature values */
    uint8_t val[2] = {0};
    int ret_read = I2C_Read_Multiple_Reg(mpu_conf->conf.i2c, mpu_conf->addr, MPU6050_TEMP_OUT_H, sizeof(val), val);
    if (ret_read != sizeof(val))
    {
        ERROR("Read temperature registers failed");
        goto end_mpu6050_readtemp;
    }

    /* Temperature Conversion */
    int16_t raw_temp = (int16_t)((val[0] << 8) | val[1]);
    double output_temp = ((double)raw_temp)/CONST_TEMP;
    output_temp += OFFSET_TEMP;

    /* Store in the output buffer */
    *temperature = output_temp;

    ret = 0;

end_mpu6050_readtemp:
    return ret;
}

void MPU6050_SetOffset(eMPU6050_BASE mpu, accel_t *accel, gyro_t *gyro)
{
    /* Get MPU configuration */
    tMPU6050_Conf *mpu_conf;
    GET_CONF(mpu_conf, mpu, end_mpu6050_setoffset);

    /* Store accelerometer offset */
    if (accel != NULL)
    {
        mpu_conf->conf.accelOffset = (*accel);
    }

    /* Store gyroscope offset */
    if (gyro != NULL)
    {
        mpu_conf->conf.gyroOffset = (*gyro);
    }

end_mpu6050_setoffset:
    return;
}

static double ConvertIMUVal(uint16_t val, double constant)
{
    double ret = 0;
    if (!(val & 0x8000))
    {
        ret = ((double)val) / constant;
    }
    else
    {
        val = (((~val) + 1) & 0x7FFF);
        ret = -(((double)val) / constant);
    }
    return ret;
}

static void ApplyAccelOffset(accel_t offset, accel_t *accel)
{
    /* Pointer protection */
    if (accel != NULL)
    {
        (*accel).x += offset.x;
        (*accel).y += offset.y;
        (*accel).z += offset.z;
    }
}

static void ApplyGyroOffset(gyro_t offset, gyro_t *gyro)
{
    /* Pointer protection */
    if (gyro != NULL)
    {
        (*gyro).x += offset.x;
        (*gyro).y += offset.y;
        (*gyro).z += offset.z;
    }
}

inline static tMPU6050_Conf *GetMPU6050Conf(eMPU6050_BASE mpu)
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