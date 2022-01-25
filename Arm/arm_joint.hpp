#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "joint.hpp"

namespace sjsu::arm
{
// the ArmJoint class is used for the rotunda, elbow, and shoulder motors.
class ArmJoint : public Joint
{
 public:
  ArmJoint(sjsu::RmdX & joint_motor, sjsu::Mpu6050 & accelerometer)
      : Joint(accelerometer), motor_(joint_motor){};

  ArmJoint(sjsu::RmdX & joint_motor,
           sjsu::Mpu6050 & accelerometer,
           float min_angle,
           float max_angle,
           float standby_angle)
      : Joint(accelerometer),
        motor_(joint_motor),
        kMinimumAngle(min_angle),
        kMaximumAngle(max_angle),
        kRestAngle(standby_angle){};

  void Initialize()
  {
    motor_.Initialize();
    mpu_.Initialize();
  }

  /// Move the motor to the (calibrated) angle desired.
  void SetPosition(float angle)
  {
    angle += offset_angle_;
    position_ = std::clamp(angle, kMinimumAngle, kMaximumAngle);
    units::angle::degree_t angle_to_degrees(position_);
    motor_.SetAngle(angle_to_degrees);
  }

  /// Sets the offset angle so motor can find its true zero
  void SetZeroOffset(float offset)
  {
    offset_angle_ = offset;
  }
  // return is only used for testing
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

  void SetJointSpeed(float target_speed)
  {
    speed_ = std::clamp(target_speed, -kMaxSpeed, kMaxSpeed);
    units::angular_velocity::revolutions_per_minute_t speed_to_rpm(speed_);
    motor_.SetSpeed(speed_to_rpm);
  }

  int GetSpeed()
  {
    return int(speed_);
  }

  int GetPosition()
  {
    return int(position_);
  }

  int GetOffsetAngle()
  {
    return int(offset_angle_);
  }

 private:
  sjsu::RmdX & motor_;

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