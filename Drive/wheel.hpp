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
    home_offset_angle = 0;
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
    return hub_speed_;
  };

  /// Gets the angle/position of the steering motor.
  int GetPosition()
  {
    return steer_angle_;
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  void SetHubSpeed(double hub_speed)
  {
    hub_speed  = std::clamp(hub_speed, -kMaxSpeed, kMaxSpeed);
    hub_speed_ = hub_speed;
    units::angular_velocity::revolutions_per_minute_t hub_spd_to_rpm(hub_speed);
    hub_motor_.SetSpeed(hub_spd_to_rpm);
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  void SetSteeringAngle(double rotation_angle)
  {
    // TODO: Add logic for offset
    steer_angle_ = rotation_angle;
    // units::angle::degree_t steer_angle(rotation_angle +
    // home_offset_angle);
    units::angle::degree_t angle_into_deg(rotation_angle);
    steer_motor_.SetAngle(angle_into_deg, kSteerSpeed);
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  void HomeWheel()
  {
    sjsu::Button homing_button(homing_pin_);
    homing_button.Initialize();

    sjsu::LogWarning("Homing %s wheel...", name_.c_str());
    SetSteeringAngle(0_deg);
    while (homing_pin_.Read() != kHomeLevel)
    {
      sjsu::LogWarning("Press %s wheel homing button when homed...", name_);
      sjsu::Delay(1s);
    }
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  void SlipRingHomeWheel()
  {
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    for (int angle = 0; angle < 360; angle += 2)
    {
      SetSteeringAngle(angle);
      sjsu::Delay(50ms);  // Lets motor move into place
      if (homing_pin_.Read() == kHomeLevel)
      {
        home_offset_angle = angle;
        break;
      }
    }
    sjsu::LogInfo("%s wheel offset angle: %d", name_.c_str(),
                  home_offset_angle);
  };

  std::string name_;
  int home_offset_angle;
  sjsu::RmdX & hub_motor_;
  sjsu::RmdX & steer_motor_;
  sjsu::Gpio & homing_pin_;
  double steer_angle_ = 0;
  double hub_speed_   = 0;

  const bool kHomeLevel     = sjsu::Gpio::kHigh;
  const double kMaxRotation = 360;
  const double kMaxSpeed    = 100;
  const double              = 0;
  const double kSteerSpeed  = 10;
};
}  // namespace sjsu::drive
