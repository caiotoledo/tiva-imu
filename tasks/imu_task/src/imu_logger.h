#ifndef MPU6050_LOGGER_H_
#define MPU6050_LOGGER_H_

/**
 * @brief IMU Log Task
 *
 * @param pvParameters Receive Queue Handler (#QueueHandle_t) to receive data (#dataIMU_t)
 */
void vIMULogTask(void *pvParameters);

#endif /* MPU6050_LOGGER_H_ */
