#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <timers.h>

extern uint32_t GetMillis(void);

void vMPU6050Task(void *pvParameters)
{
    int ret = MPU6050_Enable(MPU6050_LOW, I2C1, GetMillis);
    if (ret != 0)
    {
        ERROR("MPU6050 Error, Suspend task!");
        /* Stop this Task */
        vTaskSuspend(NULL);
    }

    for(;;)
    {
        int ret = MPU6050_Probe(MPU6050_LOW);
        if (ret == 0)
        {
            accel_t val;
            ret = MPU6050_ReadAllAccel(MPU6050_LOW, &val);
            if (ret == 0)
            {
                INFO("[%d] - X[%04d] Y[%04d] Z[%04d]", GetMillis(), (int32_t) val.x, (int32_t) val.y, (int32_t) val.z);
            }
        }
        else
        {
            ERROR("MPU6050 NOT Present!");
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
