#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <log.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <tasks_config.h>

#include "imu_types.h"
#include "imu_task.h"
#include "imu_internal.h"
#include "imu_logger.h"
#include "imu_cmd.h"

/* Size of Queue IMU for data logging */
#define QUEUE_IMU_LENGTH    (10U)

static imuTaskConfig_t taskConfig[] =
{
    {
        .name = "MPU6050 Low",
        .mpu = MPU6050_LOW,
        .i2c= I2C1,
        .gpio_int = GPIO_PA5,
        .queue = NULL,
        .taskHandler = NULL,
        .xLastWakeTime = 0U,
    },
    {
        .name = "MPU6050 High",
        .mpu = MPU6050_HIGH,
        .i2c= I2C1,
        .gpio_int = GPIO_PA4,
        .queue = NULL,
        .taskHandler = NULL,
        .xLastWakeTime = 0U,
    },
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
        /* CREATE MPU6050 Task */
        if (xTaskCreate(vMPU6050Task, "MPU6050 Task", TASK_IMU_STACKSIZE, (void *) &taskConfig[i], TASK_IMU_PRIORITY, &taskConfig[i].taskHandler) != pdPASS)
        {
            ERROR("Task [%s] NOT created!", taskConfig[i].name);
        }
        else
        {
            INFO("Task [%s] created!", taskConfig[i].name);
        }
    }

    /* Suspend all MPU6050 Tasks (It should be resume via command) */
    ManageMPU6050Tasks(false);

    /* Register IMU Commands */
    vIMU_ConfigureCommands();

end_imu_task:
    /* This task can be deleted now */
    vTaskDelete(NULL);
}

void ManageMPU6050Tasks(bool start)
{
    char state[15] = { 0 };
    if (start)
    {
        strcpy(state, "resumed");
    }
    else
    {
        strcpy(state, "suspended");
    }

    for (size_t i = 0; i < sizeof(taskConfig)/sizeof(taskConfig[0]); i++)
    {
        if (taskConfig[i].taskHandler != NULL)
        {
            bool stChange = false; /* Just to be used to LOG if there was a state change */
            eTaskState stTask = eTaskGetState(taskConfig[i].taskHandler);
            if (start)
            {
                if (
                    stTask != eRunning &&
                    stTask != eReady &&
                    stTask != eBlocked &&
                    stTask != eDeleted
                )
                {
                    /* Update tick counter to avoid problems in delay */
                    taskConfig[i].xLastWakeTime = xTaskGetTickCount();
                    vTaskResume(taskConfig[i].taskHandler);
                    stChange = true;
                }
            }
            else
            {
                if (
                    stTask != eSuspended &&
                    stTask != eDeleted
                )
                {
                    vTaskSuspend(taskConfig[i].taskHandler);
                    stChange = true;
                }
            }
            if (stChange)
            {
                INFO("Task [%s] %s!", taskConfig[i].name, state);
            }
        }
    }
}
