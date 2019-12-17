#ifndef MPU6050_TASK_H_
#define MPU6050_TASK_H_

/**
 * @brief Initialize and sample the data from IMU
 *
 * @param pvParameters Not used, should be NULL
 */
void vIMUTask(void *pvParameters);

#endif /* MPU6050_TASK_H_ */
