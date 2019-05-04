#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <timers.h>

extern uint32_t GetMillis(void);

bool bIMUEnabled = false;

void vMPU6050Task(TimerHandle_t xTimer)
{
    uint32_t ticks = GetMillis();
    INFO("TIME %d ms", ticks);

    if (!bIMUEnabled)
    {
        int ret = MPU6050_Enable(MPU6050_LOW, I2C1, GetMillis);
        if (ret != 0)
        {
            ERROR("MPU6050 Error, Suspend task!");
            /* Stop this Task */
            xTimerStop(xTimer, 0);
            return;
        }
        bIMUEnabled = true;
    }

    accel_t accel;
    int ret = MPU6050_ReadAllAccel(MPU6050_LOW, &accel);
    if (ret == 0)
    {
        INFO("[Accel] X[%05d] Y[%05d] Z[%05d]", (int32_t)accel.x, (int32_t)accel.y, (int32_t)accel.z);
    }
    gyro_t gyro;
    ret = MPU6050_ReadAllGyro(MPU6050_LOW, &gyro);
    if (ret == 0)
    {
        INFO("[Gyro] X[%05d] Y[%05d] Z[%05d]", (int32_t)gyro.x, (int32_t)gyro.y, (int32_t)gyro.z);
    }
}
