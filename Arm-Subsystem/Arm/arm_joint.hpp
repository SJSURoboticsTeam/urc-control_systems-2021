#pragma once
#include "joint_interface.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
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

  void Initialize() override
  {
    motor_.Initialize();
    Joint::Initialize();
  }

  // Move the motor to the (calibrated) angle desired.
  void SetPosition(float angle) override
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

  int GetSpeed() const override
  {
    return static_cast<int>(speed_);
  }

  int GetPosition() const override
  {
    return static_cast<int>(position_);
  }

  float GetOffsetAngle() const
  {
    return offset_angle_;
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