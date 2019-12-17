#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <timer.h>

#include <tasks_config.h>

#include "imu_types.h"
#include "imu_task.h"
#include "imu_internal.h"
#include "imu_logger.h"

/* Size of Queue IMU for data logging */
#define QUEUE_IMU_LENGTH    (10U)

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
