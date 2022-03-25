#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "../Common/accelerometer.hpp"

namespace sjsu::arm
{
class ArmJoint  
{
 public:
  ArmJoint(sjsu::RmdX & joint_motor, sjsu::Mpu6050 & accelerometer)
      : accelerometer_(accelerometer), motor_(joint_motor){};

  ArmJoint(sjsu::RmdX & joint_motor,
           sjsu::Mpu6050 & accelerometer,
           float min_angle,
           float max_angle,
           float standby_angle)
      : accelerometer_(accelerometer),
        motor_(joint_motor),
        kMinimumAngle(min_angle),
        kMaximumAngle(max_angle),
        kRestAngle(standby_angle){};

  void Initialize() 
  {
    motor_.Initialize();
    accelerometer_.Initialize();
  }

  // Move the motor to the (calibrated) angle desired.
  void SetPosition(float angle) 
  {
    angle += offset_angle_;
    position_ = std::clamp(angle, kMinimumAngle, kMaximumAngle);
    units::angle::degree_t angle_to_degrees(position_);
    motor_.SetAngle(angle_to_degrees);
  }

  // Sets the offset angle so motor can find its true zero
  void SetZeroOffset(float offset)
  {
    offset_angle_ = offset;
  }
  void SetJointSpeed(float target_speed)
  {
    speed_ = std::clamp(target_speed, -kMaxSpeed, kMaxSpeed);
    units::angular_velocity::revolutions_per_minute_t speed_to_rpm(speed_);
    motor_.SetSpeed(speed_to_rpm);
  }

  int GetSpeed() const 
  {
    return static_cast<int>(speed_);
  }

  int GetPosition() const 
  {
    return static_cast<int>(position_);
  }

  float GetOffsetAngle() const
  {
    return offset_angle_;
  }

void GetAccelerometerData()
{
  accelerometer_.GetAccelerometerData();
}
 sjsu::common::Accelerometer::Acceleration ReadAccelerometerData()
{ 
  accelerometer_.GetAccelerometerData();
  return accelerometer_.acceleration_;
}
 private:
  sjsu::RmdX & motor_;
  sjsu::common::Accelerometer accelerometer_;

  float offset_angle_ = 0;
  float speed_        = 0;
  float position_     = 0;

  const float kLerpStep     = .5;
  const float kMinimumAngle = 0;
  const float kMaximumAngle = 180;
  const float kRestAngle    = 0;
  const float kMaxSpeed     = 100;
};
}  // namespace sjsu::arm