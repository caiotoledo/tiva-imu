#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <mpu6050.h>

#include <log.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <timers.h>

/* Size of Queue IMU for data logging */
#define QUEUE_IMU_LENGTH    10

#define ABS(x)      x < 0 ? (-x) : (x)

typedef struct
{
    uint32_t ms;
    accel_t accel;
    gyro_t gyro;
} dataIMU_t;

extern uint32_t GetMillis(void);
static inline void vDouble2IntFrac(double input, int *integer, uint32_t *fraction, uint8_t precision);
static void vInitQueueIMU(QueueHandle_t *queue);

static QueueHandle_t xQueueIMU = NULL;

void vMPU6050Task(TimerHandle_t xTimer)
{
    int ret = 0;
    static bool bIMUEnabled = false;

    if (!bIMUEnabled)
    {
        /* Initialize IMU Data Queue */
        vInitQueueIMU(&xQueueIMU);

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

    dataIMU_t dataimu = { 0 };
    /* Store sample time in ms */
    dataimu.ms = GetMillis();
    /* Sample Accelerometer */
    ret += MPU6050_ReadAllAccel(MPU6050_LOW, &dataimu.accel);
    /* Sample Gyroscope */
    ret += MPU6050_ReadAllGyro(MPU6050_LOW, &dataimu.gyro);

    /* Check if the data was successful sample */
    if (ret == 0)
    {
        if (xQueueSend(xQueueIMU, (void *)&dataimu, (500/portTICK_RATE_MS)) != pdTRUE)
        {
            ERROR("Full queue!");
        }
    }
    else
    {
        ERROR("Error IMU Read!");
    }
}

void vIMULogTask(void *pvParameters)
{
    vInitQueueIMU(&xQueueIMU);
    /* Suspend task if no queue was created */
    if (xQueueIMU == NULL)
    {
        vTaskSuspend(NULL);
    }

    for(;;)
    {
        dataIMU_t data;
        /* Receive IMU data from Queue */
        if (xQueueReceive(xQueueIMU, &(data), portMAX_DELAY) != pdFALSE)
        {
            int integer[3];
            uint32_t frac[3];

            INFO("TIME %d ms", data.ms);

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
        }
    }
}

static void vInitQueueIMU(QueueHandle_t *queue)
{
    if ( (*queue) == NULL)
    {
        (*queue) = xQueueCreate(QUEUE_IMU_LENGTH, sizeof(dataIMU_t));
    }
}

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
