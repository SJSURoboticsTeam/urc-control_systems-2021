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
    // homing_pin_.GetPin().settings.Floating();  // testing with slip ring
    homing_pin_.GetPin().settings.PullDown();  // testing with button
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  int GetHubSpeed()
  {
    return int(hub_speed_);
  };

  int GetSteerAngle()
  {
    return int(steer_angle_);
  };

  /// Adjusts only the hub motor speed
  void SetHubSpeed(double hub_speed)
  {
    hub_speed_ = float(std::clamp(hub_speed, -kMaxSpeed, kMaxSpeed));
    units::angular_velocity::revolutions_per_minute_t hub_speed_rpm(hub_speed_);
    hub_motor_.SetSpeed(hub_speed_rpm);
  }

  /// Adjusts only the steer motor
  void SetSteerAngle(double steer_angle)
  {
    steer_angle_ = float((int(steer_angle) % kMaxRotation) + home_offset_angle);
    units::angle::degree_t steer_angle_degree(steer_angle_);
    steer_motor_.SetAngle(steer_angle_degree, kSteerSpeed);
  };

  /// Sets steer motor to start position - press SJ2 button when done homing
  void HomeWheel()
  {
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    sjsu::Button homing_button(homing_pin_);
    homing_button.Initialize();

    SetSteerAngle(0);
    printf("Press SJ2 %s wheel button when homed", name_.c_str());
    while (homing_pin_.Read() != kHomeLevel)
    {
      printf(".");
      sjsu::Delay(500ms);
    }
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring
  void SlipRingHomeWheel()
  {
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    for (int angle = 0; angle < 360; angle += 2)
    {
      if (homing_pin_.Read() == kHomeLevel)
      {
        home_offset_angle = angle;
        break;
      }
      SetSteerAngle(angle);
      sjsu::Delay(50ms);
    }
    sjsu::LogInfo("%s wheel offset: %d", name_.c_str(), home_offset_angle);
  };

  std::string name_     = "";
  int home_offset_angle = 0;
  float steer_angle_    = 0;
  float hub_speed_      = 0;

  sjsu::RmdX & hub_motor_;
  sjsu::RmdX & steer_motor_;
  sjsu::Gpio & homing_pin_;

  const bool kHomeLevel   = sjsu::Gpio::kHigh;
  const int kMaxRotation  = 360;
  const double kMaxSpeed  = 100;
  const double kZeroSpeed = 0;
  const units::angular_velocity::revolutions_per_minute_t kSteerSpeed = 10_rpm;
};
}  // namespace sjsu::drive
