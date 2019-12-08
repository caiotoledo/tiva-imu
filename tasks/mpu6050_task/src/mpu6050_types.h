#ifndef MPU6050_TYPES_H_
#define MPU6050_TYPES_H_

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
    QueueHandle_t queue;
} imuTaskConfig_t;

#endif /* MPU6050_TYPES_H_ */
