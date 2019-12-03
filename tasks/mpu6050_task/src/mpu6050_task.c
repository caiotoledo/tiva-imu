#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

#include <timer.h>

#include <tasks_config.h>
#include "mpu6050_task.h"

/* Size of Queue IMU for data logging */
#define QUEUE_IMU_LENGTH    (10U)

#define ABS(x)      (x < 0 ? (-x) : (x))

typedef struct
{
    char *imu;
    uint32_t ms;
    accel_t accel;
    gyro_t gyro;
    double temperature;
} dataIMU_t;

typedef struct
{
    char *name;
    eMPU6050_BASE mpu;
    eI2C_BASE i2c;
    QueueHandle_t queue;
} imuTaskConfig_t;

static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision);
static void vMPU6050Task(TimerHandle_t xTimer);
static void vIMULogTask(void *pvParameters);

static imuTaskConfig_t taskConfig[] =
{
    { .name = "MPU6050 Low",    .mpu = MPU6050_LOW,     .i2c= I2C1, .queue = NULL},
    { .name = "MPU6050 High",   .mpu = MPU6050_HIGH,    .i2c= I2C1, .queue = NULL},
};

void vIMUTask(void *pvParameters)
{
    /* INITIALIZE IMU DATA QUEUE */
    QueueHandle_t xIMUQueue = xQueueCreate(QUEUE_IMU_LENGTH, sizeof(dataIMU_t));
    if (xIMUQueue == NULL)
    {
        ERROR("Create Queue IMU data error!");
        /* Do not go further if the queue couldn't be created */
        goto end_imu_task;
    }
    INFO("Queue for IMU Data created!");

    /* CREATE IMU LOG TASK */
    /* IMU Data Queue as parameter */
    if (xTaskCreate(vIMULogTask, "IMU Log Task", TASK_IMULOG_STACKSIZE, &xIMUQueue, TASK_IMULOG_PRIORITY, NULL) != pdPASS)
    {
        ERROR("IMU Log Task Create Error!");
        /* Do not go further if the log task couldn't be created */
        goto end_imu_task;
    }
    INFO("Task IMU Log created!");

    for (size_t i = 0; i < sizeof(taskConfig)/sizeof(taskConfig[0]); i++)
    {
        /* Store the Queue Handler in task configuration */
        taskConfig[i].queue = xIMUQueue;
        if (MPU6050_Enable(taskConfig[i].mpu, taskConfig[i].i2c, GetMillis) == 0)
        {
            /* START IMU SAMPLE TIMER */
            TimerHandle_t xTimerMPU6050 = xTimerCreate("MPU6050 Timer", TASK_MPU6050_PERIOD, pdTRUE, (void *) &taskConfig[i], vMPU6050Task);
            if ( (xTimerMPU6050 == NULL) || (xTimerStart(xTimerMPU6050, 0) != pdPASS) )
            {
                ERROR("Timer [%s] NOT created!");
                /* Do not go further if the timer couldn't be created */
                continue;
            }
            INFO("Timer [%s] created!", taskConfig[i].name);
        }
        else
        {
            ERROR("[%s] Enable Error!", taskConfig[i].name);
        }
    }

end_imu_task:
    /* This task can be suspended now */
    vTaskSuspend(NULL);
}

/**
 * @brief Task to sample MPU6050 IMU
 *
 * @param xTimer Handler to provide #imuTaskConfig_t configuration
 */
static void vMPU6050Task(TimerHandle_t xTimer)
{
    int ret = 0;

    imuTaskConfig_t taskParam = *(( imuTaskConfig_t * ) pvTimerGetTimerID( xTimer ));

    /* Initialize the Mutex for this task only once */
    static SemaphoreHandle_t mtxIMU = NULL;
    if (mtxIMU == NULL)
    {
        mtxIMU = xSemaphoreCreateMutex();
    }

    /* Store sample time in ms */
    uint32_t time_ms = GetMillis();

    /* Lock IMU mutex */
    xSemaphoreTake(mtxIMU, portMAX_DELAY);
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

/**
 * @brief IMU Log Task
 *
 * @param pvParameters Receive Queue Handler (#QueueHandle_t) to receive data (#dataIMU_t)
 */
static void vIMULogTask(void *pvParameters)
{
    QueueHandle_t xQueueDataIMU = *((QueueHandle_t *)pvParameters);

    for(;;)
    {
        dataIMU_t data;
        /* Receive IMU data from Queue */
        if (xQueueReceive(xQueueDataIMU, &(data), portMAX_DELAY) != pdFALSE)
        {
            int integer[3];
            uint32_t frac[3];

            INFO("[%s] TIME %d ms", data.imu, data.ms);

            /* Convert Accel Double values for print */
            vDouble2IntFrac(data.accel.x, &integer[0], &frac[0], 4U);
            vDouble2IntFrac(data.accel.y, &integer[1], &frac[1], 4U);
            vDouble2IntFrac(data.accel.z, &integer[2], &frac[2], 4U);
            INFO("[Accel] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);

            /* Convert Gyro Double values for print */
            vDouble2IntFrac(data.gyro.x, &integer[0], &frac[0], 4U);
            vDouble2IntFrac(data.gyro.y, &integer[1], &frac[1], 4U);
            vDouble2IntFrac(data.gyro.z, &integer[2], &frac[2], 4U);
            INFO("[Gyro] X[%d.%04u] Y[%d.%04u] Z[%d.%04u]", integer[0], frac[0], integer[1], frac[1], integer[2], frac[2]);

            /* Convert Temperature Double values for print */
            vDouble2IntFrac(data.temperature, &integer[0], &frac[0], 4U);
            INFO("[Temperature] [%d.%04u]", integer[0], frac[0]);
        }
    }
}

/**
 * @brief Convert Double values in Integer and Decimal values
 *
 * @param input Double value to be converted
 * @param integer Integer part of the input
 * @param fraction Decimal part of the input
 * @param precision Precision of the decimal fraction output
 */
static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision)
{
    /* Extract signal */
    bool signal = (input < 0) ? false : true;

    /* Extract integer part */
    input = ABS(input);
    int tmpInt = input;

    /* Extract fraction part in float */
    double tmpFloatFrac = input - (double)tmpInt;
    /* Running power of 10 precision */
    double valPrec = 1;
    for (int i = 0; i < precision; i++)
    {
        valPrec *= 10;
    }
    /* Get fraction value based on the precision */
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