#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

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
        /* START MPU6050 Task */
        if (xTaskCreate(vMPU6050Task, "MPU6050 Task", TASK_IMU_STACKSIZE, (void *) &taskConfig[i], TASK_IMU_PRIORITY, NULL) != pdPASS)
        {
            ERROR("Task [%s] NOT created!", taskConfig[i].name);
        }
        else
        {
            INFO("Task [%s] created!", taskConfig[i].name);
        }
    }

end_imu_task:
    /* This task can be suspended now */
    vTaskSuspend(NULL);
}
