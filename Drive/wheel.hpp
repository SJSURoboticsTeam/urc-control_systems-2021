#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"
#include "devices/switches/button.hpp"

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
    // homing_pin_.GetPin().settings.Floating();  // for real
    homing_pin_.GetPin().settings.PullDown();  // for testing
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
    return steer_angle_.to<int>();
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  /// @param hub_speed the new speed of the wheel
  void SetHubSpeed(int hub_speed)
  {
    units::angular_velocity::revolutions_per_minute_t hub_spd_to_rpm(hub_speed);
    hub_speed_ = hub_spd_to_rpm;
    hub_motor_.SetSpeed(hub_speed_);
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  /// @param rotation_angle positive angle (turn right), negative angle (left)
  void SetSteeringAngle(int rotation_angle)
  {
    // units::angle::degree_t steer_angle(rotation_angle + homing_offset_angle_);
    units::angle::degree_t angle_into_deg(rotation_angle);
    steer_angle_ = angle_into_deg;
    steer_motor_.SetAngle(steer_angle_, kSteerSpeed);
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  /// The mark is indicated by the GPIO being set to low
  void HomeWheel()
  {
    // For testing homing procedure
    sjsu::Button homing_button(homing_pin_);
    homing_button.Initialize();

    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    // Increments through all possible angles (0-360)
    // When the homing pin is high stop incrementing and update homing offset
    bool home_level = sjsu::Gpio::kHigh;
    for (int angle = 0; angle < 360; angle += 2)
    {
      SetSteeringAngle(angle);
      sjsu::Delay(50ms);  // Lets motor move into place
      if (homing_pin_.Read() == home_level)
      {
        homing_offset_angle_ = angle;
        break;
      }
    }
    sjsu::LogInfo("Homing %s wheel done! Offset angle set to %d", name_.c_str(),
                  homing_offset_angle_);
  };

  std::string name_;          // Wheel name (i.e. left, right, back)
  sjsu::RmdX & hub_motor_;    // Controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  // Controls wheel alignment/angle
  int homing_offset_angle_                                            = 0;
  units::angle::degree_t steer_angle_                                 = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_        = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed  = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kSteerSpeed = 10_rpm;

  const units::angle::degree_t kMaxPosRotation = 360_deg;
  const units::angle::degree_t kMaxNegRotation = -360_deg;

  const units::angular_velocity::revolutions_per_minute_t kMaxPosSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNegSpeed =
      -100_rpm;
  sjsu::Gpio & homing_pin_;
};
}  // namespace sjsu::drive
