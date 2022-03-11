#pragma once
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

namespace sjsu::common
{
class Accelerometer
{
 public:
  struct Acceleration
  {
    float x = 0;
    float y = 0;
    float z = 0;
  };

  Accelerometer(sjsu::Accelerometer & accelerometer)
      : accelerometer_(accelerometer){};

  void Initialize()
  {
    accelerometer_.Initialize();
  }

  void GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(
        accelerometer_.Read());
    acceleration_.x = ChangeIfZero(static_cast<float>(acceleration_to_float.x));
    acceleration_.y = ChangeIfZero(static_cast<float>(acceleration_to_float.y));
    acceleration_.z = ChangeIfZero(static_cast<float>(acceleration_to_float.z));
  }

  float ChangeIfZero(float acceleration)
  {
    if (!(acceleration > 0 || acceleration < 0))
    {
      float nonzero_value = static_cast<float>(.001);
      return nonzero_value;
    }
    return acceleration;
  }

  sjsu::Accelerometer & accelerometer_;
  Acceleration acceleration_;
};
}  // namespace sjsu::common