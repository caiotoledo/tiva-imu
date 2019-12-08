#ifndef MPU6050_INTERNAL_H_
#define MPU6050_INTERNAL_H_

#include <timers.h>

/**
 * @brief Task to sample MPU6050 IMU
 *
 * @param xTimer Handler to provide #imuTaskConfig_t configuration
 */
void vMPU6050Task(TimerHandle_t xTimer);

#endif /* MPU6050_INTERNAL_H_ */
