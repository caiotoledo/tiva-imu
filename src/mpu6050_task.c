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

    uint32_t ticks = GetMillis();
    accel_t val;
    int ret = MPU6050_ReadAllAccel(MPU6050_LOW, &val);
    if (ret == 0)
    {
        INFO("[%d] - X[%04d] Y[%04d] Z[%04d]", ticks, (int32_t)val.x, (int32_t)val.y, (int32_t)val.z);
    }
}
