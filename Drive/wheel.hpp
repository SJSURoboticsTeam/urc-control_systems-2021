
#pragma once

#include "L2_HAL/actuators/servo/rmd_x.hpp"

namespace sjsu::drive
{
// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(sjsu::RmdX & hub_motor, sjsu::RmdX & steer_motor)
      : hub_motor_(hub_motor), steer_motor_(steer_motor)
  {
  }

  void Initialize()
  {
    hub_motor_.Initialize();
    steer_motor_.Initialize();
  }

  void Enable(bool enable = true)
  {
    hub_motor_.Enable(enable);
    steer_motor_.Enable(enable);
  }

  units::angular_velocity::revolutions_per_minute_t GetSpeed()
  {
    return hub_speed_;
  };

  units::angle::degree_t GetPosition()
  {
    return zero_offset_angle_;
  };

  void SetSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    if (hub_speed > kMaxHubSpeed)  // need use abs(hub_speed) ?
    {
      hub_speed = kMaxHubSpeed;
    }
    hub_speed_ = hub_speed;
    steer_motor_.SetSpeed(hub_speed);
  }
  // can't do steer_speed = steer_speed_ in parameter... any better solutions?
  void SetPosition(
      units::angle::degree_t rotation_angle,
      units::angular_velocity::revolutions_per_minute_t steer_speed = 20_rpm)
  {
    if (steer_speed > kMaxSteerSpeed)  // need use abs(steer_speed) ?
    {
      steer_speed = kMaxSteerSpeed;
    }
    // Assuming mission control sends angles relative to itself. Such that
    // -10_deg rotationAngle input would turn wheels slightly to the left and
    // 10_deg rotationAngle would move to the right even if zeroOffsetAngle was
    // at a non-starting/non-zero angle.
    units::angle::degree_t difference_angle =
        (rotation_angle + zero_offset_angle_);
    steer_motor_.SetAngle(difference_angle, steer_speed);
    zero_offset_angle_ += difference_angle;
  };

  void ResetZeroOffsetAngle()
  {
    SetToZeroPosition();
    zero_offset_angle_ = 0_deg;
  };

 private:
  void SetToZeroPosition(){
    // Turn the wheel until it lines back up at initial zero position.
    // Does not need to move completely back i.e -600_deg
  };

  sjsu::RmdX & hub_motor_;    // controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  // controls wheel alignment/angle
  // holds angle difference from starting/zero angle
  units::angle::degree_t zero_offset_angle_                      = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_   = 0_rpm;
  units::angular_velocity::revolutions_per_minute_t steer_speed_ = 20_rpm;

  const units::angular_velocity::revolutions_per_minute_t kMaxHubSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxSteerSpeed =
      50_rpm;
};
}  // namespace sjsu::drive