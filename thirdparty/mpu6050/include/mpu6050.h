#ifndef MPU6050_H_
#define MPU6050_H_

#include <hal_i2c.h>

typedef enum {
    MPU6050_LOW,
    MPU6050_HIGH,
} eMPU6050_BASE;

typedef struct
{
    double x;
    double y;
    double z;
} accel_t;


/**
 * @brief Enable MPU6050
 *
 * @param mpu MPU Interface (see #eMPU6050_BASE)
 * @param i2c I2C Interface (see #eI2C_BASE)
 * @param func Pointer function to provide the ms counter
 * @return int Return 0 if successful
 */
int MPU6050_Enable(eMPU6050_BASE mpu, eI2C_BASE i2c, uint32_t (*func)());

/**
 * @brief Verify MP6050 Presence in the bus
 *
 * @param mpu MPU Interface (see #eMPU6050_BASE)
 * @return int Return 0 if successful
 */
int MPU6050_Probe(eMPU6050_BASE mpu);

/**
 * @brief Return each axis acceleration
 *
 * @param mpu MPU Interface (see #eMPU6050_BASE)
 * @param accel Pointer to output struct (see #accel_t)
 * @return int Return 0 if successful
 */
int MPU6050_ReadAllAccel(eMPU6050_BASE mpu,  accel_t *accel);

#endif /* MPU6050_H_ */
