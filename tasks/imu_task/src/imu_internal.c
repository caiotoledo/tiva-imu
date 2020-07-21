#include <FreeRTOS.h>
#include <semphr.h>

#include <log.h>

#include <timer.h>

#include "imu_types.h"
#include "imu_internal.h"

/* TODO: Move this dependencies to another library */
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#define IMU_SAMPLE_RATE         (1000/portTICK_RATE_MS)

static void MPU6050_DataReady_Cb(eMPU6050_BASE mpu);

static SemaphoreHandle_t xIMUDataReadySem[2] = { NULL };

void vMPU6050Task(void *pvParameters)
{
    int ret = 0;

    imuTaskConfig_t taskParam = *(( imuTaskConfig_t * ) pvParameters);

    /* Initialize the Mutex for this task only once */
    static SemaphoreHandle_t mtxIMU = NULL;
    if (mtxIMU == NULL)
    {
        mtxIMU = xSemaphoreCreateMutex();
    }

    /* Lock IMU mutex */
    xSemaphoreTake(mtxIMU, portMAX_DELAY);
    if (MPU6050_Enable(taskParam.mpu, taskParam.i2c, GetMillis) != 0)
    {
        ERROR("[%s] Enable Error!", taskParam.name);
        /* Release IMU mutex */
        xSemaphoreGive(mtxIMU);
        goto end_mpu6050_task;
    }
    /* Release IMU mutex */
    xSemaphoreGive(mtxIMU);

    /* Initialize the Interrupt GPIO */
    int retGpioInt = MPU6050_ConfigInterrupt(taskParam.mpu, taskParam.gpio_int, MPU6050_DataReady_Cb);
    if (retGpioInt != 0)
    {
        ERROR("[%s] Interrupt Configuration Error!", taskParam.name);
        goto end_mpu6050_task;
    }

    /* Attempt to create a semaphore. */
    if (xIMUDataReadySem[taskParam.mpu] == NULL)
    {
        xIMUDataReadySem[taskParam.mpu] = xSemaphoreCreateBinary();
    }

    /* Timer initialization */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /* Wait IMU Data to be ready */
        if (xSemaphoreTake(xIMUDataReadySem[taskParam.mpu], (IMU_SAMPLE_RATE/10)) != pdTRUE)
        {
            WARN("[%s] IMU Data not Ready!", taskParam.name);
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
            /* Send IMU data via queue */
            if (xQueueSend(taskParam.queue, (void *)&dataimu, portMAX_DELAY) != pdTRUE)
            {
                ERROR("Full queue!");
            }
        }
        else
        {
            ERROR("Error IMU Read!");
        }

        vTaskDelayUntil(&xLastWakeTime, (TickType_t)IMU_SAMPLE_RATE);
    }

end_mpu6050_task:
    ERROR("Suspend %s Task!", taskParam.name);
    vTaskSuspend(NULL);
}

static void MPU6050_DataReady_Cb(eMPU6050_BASE mpu)
{
    /* Check if the semaphore is initialized */
    if (xIMUDataReadySem[mpu] != NULL)
    {
        xSemaphoreGiveFromISR(xIMUDataReadySem[mpu], NULL);
    }
}
