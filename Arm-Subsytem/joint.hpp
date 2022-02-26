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

  Joint(sjsu::Accelerometer & accelerometer) : accelerometer_(accelerometer){};

  void Initialize()
  {
    accelerometer_.Initialize();
  }

  void GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(accelerometer_.Read());
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
    double swap_to = .001;
    return static_cast<float>(!(acceleration > 0 || acceleration < 0) ? swap_to : static_cast<double>(acceleration));
  }

  sjsu::Accelerometer & accelerometer_;
  Acceleration acceleration_;
};
}  // namespace sjsu::arm