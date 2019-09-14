#ifndef MPU6050_TASK_H_
#define MPU6050_TASK_H_

/**
 * @brief Task to interact with IMU MPU6050
 * 
 * @param xTimer FreeRTOS Timer Handler
 */
void vMPU6050Task(TimerHandle_t xTimer);

#endif /* MPU6050_TASK_H_ */