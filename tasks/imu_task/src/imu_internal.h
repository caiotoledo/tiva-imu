#ifndef IMU_INTERNAL_H_
#define IMU_INTERNAL_H_

#include <timers.h>

/**
 * @brief Task to sample MPU6050 IMU
 *
 * @param xTimer Handler to provide #imuTaskConfig_t configuration
 */
void vMPU6050Task(void *pvParameters);

/**
 * @brief Configure the sample rate for MPU6050
 *
 * @param sample_rate Sample rate in ms
 * @return int Return 0 when successful
 */
int MPU6050Task_SetSampleRate(uint32_t sample_rate);

#endif /* IMU_INTERNAL_H_ */
