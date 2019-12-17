#include <FreeRTOS.h>
#include <semphr.h>

#include <log.h>

#include <timer.h>

#include "imu_types.h"
#include "imu_internal.h"

void vMPU6050Task(TimerHandle_t xTimer)
{
    int ret = 0;

    imuTaskConfig_t taskParam = *(( imuTaskConfig_t * ) pvTimerGetTimerID( xTimer ));

    /* Initialize the Mutex for this task only once */
    static SemaphoreHandle_t mtxIMU = NULL;
    if (mtxIMU == NULL)
    {
        mtxIMU = xSemaphoreCreateMutex();
    }

    /* Lock IMU mutex */
    xSemaphoreTake(mtxIMU, portMAX_DELAY);

    /* Store sample time in ms */
    uint32_t time_ms = GetMillis();
    /* Sample Accelerometer */
    accel_t accel;
    ret += MPU6050_ReadAllAccel(taskParam.mpu, &accel);
    /* Sample Gyroscope */
    gyro_t gyro;
    ret += MPU6050_ReadAllGyro(taskParam.mpu, &gyro);
    /* Sample Temperature */
    double temp;
    ret += MPU6050_ReadTemperature(taskParam.mpu, &temp);

    /* Release IMU mutex */
    xSemaphoreGive(mtxIMU);

    /* Check if the data was successful sample */
    if (ret == 0)
    {
        /* Store IMU Data */
        dataIMU_t dataimu = {
            .imu = taskParam.name,
            .ms = time_ms,
            .accel = accel,
            .gyro = gyro,
            .temperature = temp,
        };
        /* Wait for half of the period of the timer to the queue be available */
        TickType_t xTimerPeriod = xTimerGetPeriod(xTimer)/2;
        /* Send IMU data via queue */
        if (xQueueSend(taskParam.queue, (void *)&dataimu, xTimerPeriod) != pdTRUE)
        {
            ERROR("Full queue!");
        }
    }
    else
    {
        ERROR("Error IMU Read!");
    }
}
