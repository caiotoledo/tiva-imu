#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_

#include <mpu6050.h>

#include <FreeRTOS.h>
#include <queue.h>

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
    eMPU6050_GPIOInt gpio_int;
    QueueHandle_t queue;
    TaskHandle_t taskHandler;
    TickType_t xLastWakeTime;
    bool bRunSample;
} imuTaskConfig_t;

/**
 * @brief Start or Stop MPU6050 Tasks
 *
 * @param start Start Tasks if True, otherwise Stop Tasks
 */
void ManageMPU6050Tasks(bool start);

#endif /* IMU_TYPES_H_ */
