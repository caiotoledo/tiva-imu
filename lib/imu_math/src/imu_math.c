#include <stdint.h>
#include <math.h>

#include <imu_math.h>

#define RAD_TO_DEG(x)   (((double)x)*((double)180.0/M_PI))

int CalculatePureAngle(double *angle, eAxis axis, accel_t accel, eAngleUnit unit)
{
  int ret = 0;
  double value = 0;

  /**
   * Reference: https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/
   */
  switch (axis)
  {
    case eAxisX:
      value = atan2(-accel.y, -accel.z) + M_PI;
      break;
    case eAxisY:
      value = atan2(-accel.x, -accel.z) + M_PI;
      break;
    case eAxisZ:
      value = atan2(-accel.y, -accel.x) + M_PI;
      break;
    default:
      ret = -1;
      break;
  }

  switch (unit)
  {
  case eDegrees:
    value = RAD_TO_DEG(value);
    break;
  case eRadians:
    break;
  default:
    ret = -1;
    break;
  }

  if (angle != NULL)
  {
    (*angle) = value;
  }

  return ret;
}
