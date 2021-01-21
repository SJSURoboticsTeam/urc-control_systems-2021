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

  /// Gets the speed of the hub motor.
  double GetSpeed()
  {
    return hub_speed_.to<double>();
  };

  /// Gets the angle/position of the steering motor.
  double GetPosition()
  {
    return homing_offset_angle_.to<double>();
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  /// @param hub_speed the new speed of the wheel
  void SetSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    if (hub_speed > kMaxHubSpeed)  // need use abs(hub_speed) ?
    {
      hub_speed = kMaxHubSpeed;
    }
    if (hub_speed < kMaxNegativeHubSpeed)  // need use abs(hub_speed) ?
    {
      hub_speed = kMaxNegativeHubSpeed;
    }
    hub_speed_ = hub_speed;
    steer_motor_.SetSpeed(hub_speed);
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  /// @param rotation_angle positive angle (turn right), negative angle (left)
  /// @param steer_speed speed to adjust steering motor by - might remove
  void SetPosition(
      units::angle::degree_t rotation_angle,
      units::angular_velocity::revolutions_per_minute_t steer_speed = 20_rpm)
  {
    if (steer_speed > kMaxSteerSpeed)
    {
      steer_speed = kMaxSteerSpeed;
    }
    units::angle::degree_t difference_angle =
        (rotation_angle + homing_offset_angle_);
    steer_motor_.SetAngle(difference_angle, steer_speed);
    homing_offset_angle_ += difference_angle;
  };

  // Sets the wheel back in its homing position by finding mark in slip ring
  void HomeWheel()
  {
    SetWheelToZeroPosition();
    homing_offset_angle_ = 0_deg;
  };

 private:
  /// Finds the homing slip ring mark. *Possibly by inverting
  /// homing_offset_angle_
  void SetWheelToZeroPosition();

  sjsu::RmdX & hub_motor_;    /// controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  /// controls wheel alignment/angle
  units::angle::degree_t homing_offset_angle_                    = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_   = 0_rpm;
  units::angular_velocity::revolutions_per_minute_t steer_speed_ = 20_rpm;

  const units::angular_velocity::revolutions_per_minute_t kMaxHubSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNegativeHubSpeed =
      -100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxSteerSpeed =
      50_rpm;
};
}  // namespace sjsu::drive
