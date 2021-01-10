
#pragma once

#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L2_HAL/actuators/servo/rmd_x.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"

namespace sjsu::drive
{
// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(sjsu::RmdX & hubMotor,
        sjsu::RmdX & steerMotor,
      : hubMotor_(hubMotor), steerMotor_(steerMotor)
  {
  }

  void Initialize()
  {
    hubMotor_.Initialize();
    steerMotor_.Initiailize();
  }

  void Enable(bool enable = true)
  {
    hubMotor_.Enable(enable);
    steerMotor_.Enable(enable);
  }

  void SetSpeed(units::angular_velocity::revolutions_per_minute_t hubSpeed)
  {
    if (hubSpeed > MAX_HUB_SPEED)
    {
      hubSpeed = MAX_HUB_SPEED;
    }
    hubSpeed_ = hubSpeed;
    steerMotor_.SetSpeed(hubSpeed);
  }

  void SetPosition(units::angle::degree_t rotationAngle, units::angular_velocity::revolutions_per_minute_t steerSpeed = steerSpeed_)
  {
    if (steerSpeed > MAX_STEER_SPEED)
    {
      steerSpeed = MAX_STEER_SPEED;
    }
    // Assuming mission control sends angles relative to itself. Such that
    // -10_deg rotationAngle input would turn wheels slightly to the left and
    // 10_deg rotationAngle would move to the right even if zeroOffsetAngle was
    // at a non-starting/non-zero angle.
    units::angle::degree_t differenceAngle = (rotationAngle + zeroOffsetAngle_);
    steerMotor_.SetAngle(differenceAngle, steerSpeed);
    zeroOffsetAngle_ += differenceAngle;
  };

 private:
  sjsu::RmdX & hubMotor_;    // controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steerMotor_;  // controls wheel alignment/angle
  
  units::angle::degree_t zeroOffsetAngle_ = 0_deg; // holds angle difference from starting/zero angle
  units::angular_velocity::revolutions_per_minute_t hubSpeed_      = 0_rpm;
  units::angular_velocity::revolutions_per_minute_t steerSpeed_    = 20_rpm;

  const units::angular_velocity::revolutions_per_minute_t MAX_HUB_SPEED   = 100_rpm;
  const units::angular_velocity::revolutions_per_minute_t MAX_STEER_SPEED   = 50_rpm;
}
}  // namespace sjsu::drive