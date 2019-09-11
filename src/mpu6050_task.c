#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <timers.h>

#define ABS(x)      x < 0 ? (-x) : (x)

extern uint32_t GetMillis(void);
static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision);


void vMPU6050Task(TimerHandle_t xTimer)
{
    int ret;
    static bool bIMUEnabled = false;
    uint32_t ticks = GetMillis();
    INFO("TIME %d ms", ticks);

    if (!bIMUEnabled)
    {
        ret = MPU6050_Enable(MPU6050_LOW, I2C1, GetMillis);
        if (ret != 0)
        {
            ERROR("MPU6050 Error, Suspend task!");
            /* Stop this Task */
            xTimerStop(xTimer, 0);
            return;
        }
        bIMUEnabled = true;
    }

    /* Sample Accelerometer */
    accel_t accel;
    ret = MPU6050_ReadAllAccel(MPU6050_LOW, &accel);
    if (ret == 0)
    {
        int integer[3];
        uint32_t frac[3];
        vDouble2IntFrac(accel.x, &integer[0], &frac[0], 4U);
        vDouble2IntFrac(accel.y, &integer[1], &frac[1], 4U);
        vDouble2IntFrac(accel.z, &integer[2], &frac[2], 4U);
        INFO("[Accel] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);
    }

    /* Sample Gyroscope */
    gyro_t gyro;
    ret = MPU6050_ReadAllGyro(MPU6050_LOW, &gyro);
    if (ret == 0)
    {
        int integer[3];
        uint32_t frac[3];
        vDouble2IntFrac(gyro.x, &integer[0], &frac[0], 4U);
        vDouble2IntFrac(gyro.y, &integer[1], &frac[1], 4U);
        vDouble2IntFrac(gyro.z, &integer[2], &frac[2], 4U);
        INFO("[Gyro] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);
    }
}

static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision)
{
    /* Extract signal */
    bool signal = (input < 0) ? false : true;

    /* Extract integer part */
    input = ABS(input);
    int tmpInt = input;

    /* Extract fraction part */
    double tmpFloatFrac = input - (double)tmpInt;
    /* Running power of 10 precision */
    double valPrec = 1;
    for (int i = 0; i < precision; i++)
    {
        valPrec *= 10;
    }
    uint32_t tmpFrac = tmpFloatFrac * valPrec;

    /* Return values */
    if (integer)
    {
        *integer = signal ? tmpInt : (-tmpInt);
    }
    if (fraction)
    {
        *fraction = tmpFrac;
    }
}
