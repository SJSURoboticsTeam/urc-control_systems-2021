#pragma once

#include "L2_HAL/actuators/servo/rmd_x.hpp"

namespace sjsu::drive
{
/// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(sjsu::RmdX & hub_motor, sjsu::RmdX & steer_motor)
      : hub_motor_(hub_motor), steer_motor_(steer_motor){};

  void Initialize()
  {
    hub_motor_.Initialize();
    steer_motor_.Initialize();
  };

  void Enable(bool enable = true)
  {
    hub_motor_.Enable(enable);
    steer_motor_.Enable(enable);
  };

  /// Return the speed of the hub motor in RPM.
  /// Look into rmd_x function RequestFeedbackFromMotor() & GetFeedback()
  units::angular_velocity::revolutions_per_minute_t GetSpeed()
  {
    return hub_speed_;
  };

  /// Return the angle/position of the steering motor.
  /// Look into rmd_x function RequestFeedbackFromMotor() & GetFeedback()
  units::angle::degree_t GetPosition()
  {
    return zero_offset_angle_;
  };

  /// Sets the speed of the hub motor and updates hub_motor_ variable so long as
  /// desired speed is under the max speed
  void SetSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    if (hub_speed > kMaxHubSpeed)  // need use abs(hub_speed) ?
    {
      hub_speed = kMaxHubSpeed;
    }
    hub_speed_ = hub_speed;
    steer_motor_.SetSpeed(hub_speed);
  }

  /// Sets the steer motor to new position/angle. Angle adjusts according to the
  /// motors current position, not absolute position. i.e. a rotation angle of
  /// 10_deg would move motor 10_deg to the right and -10_deg -> =10_deg to left
  void SetPosition(
      units::angle::degree_t rotation_angle,
      units::angular_velocity::revolutions_per_minute_t steer_speed = 20_rpm)
  {
    if (steer_speed > kMaxSteerSpeed)  // need use abs(steer_speed) ?
    {
      steer_speed = kMaxSteerSpeed;
    }
    units::angle::degree_t difference_angle =
        (rotation_angle + zero_offset_angle_);
    steer_motor_.SetAngle(difference_angle, steer_speed);
    zero_offset_angle_ += difference_angle;
  };

  // Sets the wheel back in its homing position by finding mark in slip ring
  void HomeWheel()
  {
    SetWheelToZeroPosition();
    zero_offset_angle_ = 0_deg;
  };

 private:
  void SetWheelToZeroPosition(){};

  sjsu::RmdX & hub_motor_;    /// controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  /// controls wheel alignment/angle
  units::angle::degree_t zero_offset_angle_ =
      0_deg;  /// holds angle difference from starting/zero angle
  units::angular_velocity::revolutions_per_minute_t hub_speed_   = 0_rpm;
  units::angular_velocity::revolutions_per_minute_t steer_speed_ = 20_rpm;

  const units::angular_velocity::revolutions_per_minute_t kMaxHubSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxSteerSpeed =
      50_rpm;
};
}  // namespace sjsu::drive
