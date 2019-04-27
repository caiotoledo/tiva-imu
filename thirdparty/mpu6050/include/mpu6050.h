#ifndef MPU6050_H_
#define MPU6050_H_

#include <hal_i2c.h>

typedef enum {
    MPU6050_LOW,
    MPU6050_HIGH,
} eMPU6050_BASE;

/**
 * @brief Enable MPU6050
 * 
 * @param mpu MPU Interface (see #eMPU6050_BASE)
 * @param i2c I2C Interface (see #eI2C_BASE)
 * @param func Pointer function to provide the ms counter
 * @return int Return 0 if successful
 */
int MPU6050_Enable(eMPU6050_BASE mpu, eI2C_BASE i2c, uint32_t (*func)());

#endif /* MPU6050_H_ */
