#include <FreeRTOS.h>
#include <semphr.h>

#include <log.h>

#include <timer.h>

#include "imu_types.h"
#include "imu_internal.h"

#define CONVERT_MS_TO_TICKS(x)      (x/portTICK_RATE_MS)

#define IMU_INITIAL_SAMPLE_RATE     (1000)
#define IMU_MIN_SAMPLE_RATE         (10)
#define IMU_MAX_SAMPLE_RATE         (10000)

static int MPU6050_SampleImuData(eMPU6050_BASE mpu, dataIMU_t *data);
static void MPU6050_DataReady_Cb(eMPU6050_BASE mpu);

static SemaphoreHandle_t xIMUDataReadySem[2] = { NULL };
static uint32_t u32SampleRate = CONVERT_MS_TO_TICKS(IMU_INITIAL_SAMPLE_RATE);

void vMPU6050Task(void *pvParameters)
{
    int ret = 0;

    imuTaskConfig_t *taskParam = (( imuTaskConfig_t * ) pvParameters);

    /* Initialize the Mutex for this task only once */
    static SemaphoreHandle_t mtxIMU = NULL;
    if (mtxIMU == NULL)
    {
        mtxIMU = xSemaphoreCreateMutex();
    }

    /* Lock IMU mutex */
    xSemaphoreTake(mtxIMU, portMAX_DELAY);
    if (MPU6050_Enable(taskParam->mpu, taskParam->i2c, GetMillis) != 0)
    {
        ERROR("[%s] Enable Error!", taskParam->name);
        /* Release IMU mutex */
        xSemaphoreGive(mtxIMU);
        goto end_mpu6050_task;
    }
    /* Release IMU mutex */
    xSemaphoreGive(mtxIMU);

    /* Initialize the Interrupt GPIO */
    int retGpioInt = MPU6050_ConfigInterrupt(taskParam->mpu, taskParam->gpio_int, MPU6050_DataReady_Cb);
    if (retGpioInt != 0)
    {
        ERROR("[%s] Interrupt Configuration Error!", taskParam->name);
        goto end_mpu6050_task;
    }

    /* Attempt to create a semaphore. */
    if (xIMUDataReadySem[taskParam->mpu] == NULL)
    {
        xIMUDataReadySem[taskParam->mpu] = xSemaphoreCreateBinary();
    }

    /* Timer initialization */
    taskParam->xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /* Wait IMU Data to be ready */
        if (xSemaphoreTake(xIMUDataReadySem[taskParam->mpu], (u32SampleRate/10)) != pdTRUE)
        {
            WARN("[%s] IMU Data not Ready!", taskParam->name);
        }

        /* Lock IMU mutex */
        xSemaphoreTake(mtxIMU, portMAX_DELAY);

        dataIMU_t dataimu = { 0 };
        dataimu.imu = taskParam->name;
        MPU6050_SampleImuData(taskParam->mpu, &dataimu);

        /* Release IMU mutex */
        xSemaphoreGive(mtxIMU);

        /* Check if the data was successful sample */
        if (ret == 0)
        {
            /* Send IMU data via queue */
            if (xQueueSend(taskParam->queue, (void *)&dataimu, portMAX_DELAY) != pdTRUE)
            {
                ERROR("Full queue!");
            }
        }
        else
        {
            ERROR("Error IMU Read!");
        }

        vTaskDelayUntil(&taskParam->xLastWakeTime, (TickType_t)u32SampleRate);
    }

end_mpu6050_task:
    taskParam->taskHandler = NULL;
    ERROR("Delete [%s] Task!", taskParam->name);
    vTaskDelete(NULL);
}

int MPU6050Task_SetSampleRate(uint32_t sample_rate)
{
    int ret = -1;

    if ((sample_rate >= IMU_MIN_SAMPLE_RATE) && (sample_rate <= IMU_MAX_SAMPLE_RATE))
    {
        u32SampleRate = CONVERT_MS_TO_TICKS(sample_rate);
        ret = 0;
    }

    return ret;
}

static int MPU6050_SampleImuData(eMPU6050_BASE mpu, dataIMU_t *data)
{
    int ret = 0;

    /* Check invalid pointer */
    if (data == NULL)
    {
        return -1;
    }

    /* Store sample time in ms */
    data->ms = GetMillis();
    /* Sample Accelerometer */
    ret += MPU6050_ReadAllAccel(mpu, &data->accel);
    /* Sample Gyroscope */
    ret += MPU6050_ReadAllGyro(mpu, &data->gyro);
    /* Sample Temperature */
    ret += MPU6050_ReadTemperature(mpu, &data->temperature);

    return ret;
}

static void MPU6050_DataReady_Cb(eMPU6050_BASE mpu)
{
    /* Check if the semaphore is initialized */
    if (xIMUDataReadySem[mpu] != NULL)
    {
        xSemaphoreGiveFromISR(xIMUDataReadySem[mpu], NULL);
    }
}
