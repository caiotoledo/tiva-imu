#ifndef IMU_MATH_H_
#define IMU_MATH_H_

#include <mpu6050.h>

typedef enum
{
  eAxisX,
  eAxisY,
  eAxisZ,
} eAxis;

typedef enum
{
  eRadians,
  eDegrees,
} eAngleUnit;

/**
 * @brief Calculate the tilt angle in a given axis, in degrees or radians.
 *
 * @param angle Pointer to receive the angle calculated
 * @param axis Desired Axis, ref #eAxis
 * @param accel Accelerometer value, in mG
 * @param unit Desired Unit, ref #eAngleUnit
 * @return int Returns 0 in success.
 */
int CalculatePureAngle(double *angle, eAxis axis, accel_t accel, eAngleUnit unit);

#endif /* IMU_MATH_H_ */
