#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"

namespace sjsu::drive
{
/// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(std::string name,
        sjsu::RmdX & hub_motor,
        sjsu::RmdX & steer_motor,
        sjsu::Gpio & homing_pin)
      : name_(name),
        hub_motor_(hub_motor),
        steer_motor_(steer_motor),
        homing_pin_(homing_pin){};

  void Initialize()
  {
    hub_motor_.Initialize();
    steer_motor_.Initialize();
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  /// Gets the speed of the hub motor.
  int GetSpeed()
  {
    return hub_speed_.to<int>();
  };

  /// Gets the angle/position of the steering motor.
  int GetPosition()
  {
    return homing_offset_angle_.to<int>();
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  /// @param hub_speed the new speed of the wheel
  void SetHubSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    try
    {
       
       hub_motor_.SetSpeed(hub_speed);
       hub_speed_ = hub_speed;
       
    }
    catch (const std::exception & e)
    {
      throw e;
    }
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

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  /// The mark is indicated by the GPIO being set to low
  void HomeWheel()
  {
    // TODO: Needs to be cleaned up - early prototype
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());
    bool home_level = sjsu::Gpio::kHigh;
    if (homing_pin_.Read() == home_level)
    {
      sjsu::LogInfo("Wheel %s already homed", name_.c_str());
      return;
    }

    steer_motor_.SetSpeed(10_rpm);
    while (homing_pin_.Read() != home_level)
    {
      sjsu::LogInfo("spinning");
      continue;
      // break;  // for testing purposes - comment out
    }
    steer_motor_.SetSpeed(0_rpm);
    sjsu::LogInfo("Wheel %s homed!", name_.c_str());
  };

  std::string name_;          // Wheel name (i.e. left, right, back)
  sjsu::RmdX & hub_motor_;    // Controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  // Controls wheel alignment/angle
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
  sjsu::Gpio & homing_pin_;
};
}  // namespace sjsu::drive
