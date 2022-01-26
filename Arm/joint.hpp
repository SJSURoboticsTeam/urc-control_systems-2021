#pragma once
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

namespace sjsu::arm
{
class Joint
{
 public:
  struct Acceleration
  {
    float x = 0;
    float y = 0;
    float z = 0;
  };

  Joint(sjsu::Mpu6050 & accelerometer) : mpu_(accelerometer){};

  void GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(mpu_.Read());
    acceleration_.x =
        ReturnChangedIfZero(static_cast<float>(acceleration_to_float.x));
    acceleration_.y =
        ReturnChangedIfZero(static_cast<float>(acceleration_to_float.y));
    acceleration_.z =
        ReturnChangedIfZero(static_cast<float>(acceleration_to_float.z));
  }

  /// Checks if value is zero. If it's zero make it not zero
  float ReturnChangedIfZero(float acceleration)
  {
    return (acceleration == 0 ? .001 : acceleration);
  }

  sjsu::Mpu6050 & mpu_;
  Acceleration acceleration_;
};
}  // namespace sjsu::arm