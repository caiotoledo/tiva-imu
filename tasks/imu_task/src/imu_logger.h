#ifndef IMU_LOGGER_H_
#define IMU_LOGGER_H_

/**
 * @brief IMU Log Task
 *
 * @param pvParameters Receive Queue Handler (#QueueHandle_t) to receive data (#dataIMU_t)
 */
void vIMULogTask(void *pvParameters);

#endif /* #define IMU_LOGGER_H_ */
