#ifndef IMU_INTERNAL_H_
#define IMU_INTERNAL_H_

#include <timers.h>

/**
 * @brief Task to sample MPU6050 IMU
 *
 * @param xTimer Handler to provide #imuTaskConfig_t configuration
 */
void vMPU6050Task(void *pvParameters);

#endif /* IMU_INTERNAL_H_ */
