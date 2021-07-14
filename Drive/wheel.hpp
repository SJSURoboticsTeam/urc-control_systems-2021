#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"

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
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
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
  void SetHubSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    try
    {
      sjsu::LogInfo("made it to sethubspeed()");
      // units::angular_velocity::revolutions_per_minute_t num = hub_speed / 10;
      hub_motor_.SetSpeed(hub_speed);
      auto clampedHubSpeed = std::clamp(hub_speed, kMaxNegSpeed, kMaxPosSpeed);
      // for (int i = 0; i < 10; i++)
      // {
      //   hub_motor_.SetSpeed(num);
      //   num += num;
      //   sjsu::Delay(200ms);
      // }
    }
    catch (const std::exception & e)
    {
      sjsu::LogInfo("made it to sethubspeed() - something broke!");
      throw e;
    }

    /*
    auto clampedHubSpeed = std::clamp(hub_speed, kMaxNegSpeed, kMaxPosSpeed);
    hub_motor_.SetSpeed(clampedHubSpeed);
    hub_speed_ = clampedHubSpeed;
    */
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  /// @param rotation_angle positive angle (turn right), negative angle (left)
  void SetSteeringAngle(units::angle::degree_t rotation_angle)
  {
    auto clampedRotationAngle =
        std::clamp(rotation_angle, kMaxNegRotation, kMaxPosRotation);
    units::angle::degree_t difference_angle =
        (homing_offset_angle_ + clampedRotationAngle);

    steer_motor_.SetAngle(difference_angle, kSteeringSpeed);
    homing_offset_angle_ += clampedRotationAngle;
  };

  // Sets the wheel back in its homing position by finding mark in slip ring
  void HomeWheel()
  {
    sjsu::LogInfo("homing...");
    bool home_level = sjsu::Gpio::kLow;
    if (homing_pin_.Read() == home_level)
    {
      sjsu::LogInfo("already home");
      return;
    }

    steer_motor_.SetSpeed(20_rpm);
    while (homing_pin_.Read() != home_level)
    {
      sjsu::LogInfo("spinning");
      break;  // for testing purposes - comment out
    }
    steer_motor_.SetSpeed(0_rpm);
  };

  sjsu::RmdX & hub_motor_;    /// controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  /// controls wheel alignment/angle
  units::angle::degree_t homing_offset_angle_                  = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_ = 0_rpm;

  const units::angle::degree_t kMaxPosRotation = 360_deg;
  const units::angle::degree_t kMaxNegRotation = -360_deg;
  const units::angular_velocity::revolutions_per_minute_t kMaxPosSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNegSpeed =
      -100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kSteeringSpeed =
      20_rpm;
  sjsu::Gpio & homing_pin_ = sjsu::lpc40xx::GetGpio<1, 30>();
};
}  // namespace sjsu::drive
