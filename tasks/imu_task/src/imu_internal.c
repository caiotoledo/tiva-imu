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

/* Struct Foward declaration */
typedef struct state_struct state_t;
/* Function pointer type */
typedef void (*imuFuncPtr)(state_t *);

struct state_struct
{
    imuTaskConfig_t *config;
    imuFuncPtr next;
};

/* State Machine Functions */
static void MPU6050_Initialization(state_t *state);
static void MPU6050_Run(state_t *state);

static int MPU6050_SampleImuData(eMPU6050_BASE mpu, dataIMU_t *data);
static void MPU6050_DataReady_Cb(eMPU6050_BASE mpu);

/* Mutex to control concurrent access to the I2C peripheral */
static SemaphoreHandle_t mtxI2C = NULL;
/* Mutex notification when IMU Data is ready */
static SemaphoreHandle_t xIMUDataReadySem[2] = { NULL };
/* IMU sample rate in milliseconds */
static uint32_t u32SampleRate = CONVERT_MS_TO_TICKS(IMU_INITIAL_SAMPLE_RATE);

void vMPU6050Task(void *pvParameters)
{
    state_t stMPU6050Task = { 0 };
    imuTaskConfig_t *taskParam = (( imuTaskConfig_t * ) pvParameters);

    /* Initialize I2C Mutex for this task only once */
    if (mtxI2C == NULL)
    {
        mtxI2C = xSemaphoreCreateMutex();
    }

    /* Run State Machine */
    stMPU6050Task.config = taskParam;
    stMPU6050Task.next = MPU6050_Initialization; /* Initial State */
    while (stMPU6050Task.next != NULL)
    {
        stMPU6050Task.next(&stMPU6050Task);
    }

    /* Delete task when State Machine finishes */
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

static void MPU6050_Initialization(state_t *state)
{
    /* Lock I2C mutex */
    xSemaphoreTake(mtxI2C, portMAX_DELAY);

    if (MPU6050_Enable(state->config->mpu, state->config->i2c, GetMillis) != 0)
    {
        ERROR("[%s] Enable Error!", state->config->name);
        state->next = NULL;
        goto end_mpu6050_initialization;
    }

    /* Initialize the Interrupt GPIO */
    int retGpioInt = MPU6050_ConfigInterrupt(state->config->mpu, state->config->gpio_int, MPU6050_DataReady_Cb);
    if (retGpioInt != 0)
    {
        ERROR("[%s] Interrupt Configuration Error!", state->config->name);
        state->next = NULL;
        goto end_mpu6050_initialization;
    }

    /* Attempt to create a semaphore. */
    if (xIMUDataReadySem[state->config->mpu] == NULL)
    {
        xIMUDataReadySem[state->config->mpu] = xSemaphoreCreateBinary();
    }

end_mpu6050_initialization:
    /* Release I2C mutex */
    xSemaphoreGive(mtxI2C);

    if (state->next != NULL)
    {
        /* Start IMU Sample */
        state->next = MPU6050_Run;
    }
}

static void MPU6050_Run(state_t *state)
{
    /* Wait IMU Data to be ready */
    if (xSemaphoreTake(xIMUDataReadySem[state->config->mpu], (u32SampleRate/10)) != pdTRUE)
    {
        WARN("[%s] IMU Data not Ready!", state->config->name);
    }

    /* Lock I2C mutex */
    xSemaphoreTake(mtxI2C, portMAX_DELAY);

    dataIMU_t dataimu = { 0 };
    dataimu.imu = state->config->name;
    int ret = MPU6050_SampleImuData(state->config->mpu, &dataimu);

    /* Release I2C mutex */
    xSemaphoreGive(mtxI2C);

    /* Check if the data was successful sample */
    if (ret == 0)
    {
        /* Send IMU data via queue */
        if (xQueueSend(state->config->queue, (void *)&dataimu, portMAX_DELAY) != pdTRUE)
        {
            ERROR("Full queue!");
        }
    }
    else
    {
        ERROR("Error IMU Read!");
    }

    /* Timer initialization */
    if (state->config->xLastWakeTime == 0U)
    {
        state->config->xLastWakeTime = xTaskGetTickCount();
    }
    vTaskDelayUntil(&state->config->xLastWakeTime, (TickType_t)u32SampleRate);
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
