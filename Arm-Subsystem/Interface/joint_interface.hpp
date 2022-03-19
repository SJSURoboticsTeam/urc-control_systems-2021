#pragma once
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "../Common/accelerometer.hpp"

namespace sjsu::arm
{
class Joint // interface
{

  virtual void Initialize()             = 0;
  virtual void SetPosition(float angle) = 0;
  virtual int GetSpeed()                = 0;
  virtual int GetPosition()             = 0;
  
  protected:
  sjsu::Common::Accelerometer test;
};
}  // namespace sjsu::arm