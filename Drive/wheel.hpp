#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"
//#include "../Common/gpio.hpp"
#include "devices/switches/button.hpp"
#include "string_view"

namespace sjsu::drive
{
/// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(std::string_view name,
        sjsu::RmdX & hub_motor,
        sjsu::RmdX & steer_motor,
        sjsu::Gpio & homing_pin)
      : name_(name),
        hub_motor_(hub_motor),
        steer_motor_(steer_motor),
        homing_pin_(homing_pin){};

  void Initialize()
  {
    sjsu::LogInfo("Initializing %s wheel...", name_);
    hub_motor_.Initialize();
    steer_motor_.Initialize();
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  void Print()
  {
    printf("%-10s%-10d%-10d\n", name_, GetHubSpeed(), GetSteerAngle());
  }

  std::string_view GetName() const
  {
    return name_;
  }

  int GetHubSpeed() const
  {
    return int(hub_speed_);
  };

  int GetSteerAngle() const
  {
    return int(steer_angle_);
  };

  void SetHubSpeed(float speed)
  {
    hub_speed_ = float(std::clamp(speed, -kMaxSpeed, kMaxSpeed));
    units::angular_velocity::revolutions_per_minute_t hub_speed_rpm(hub_speed_);
    hub_motor_.SetSpeed(hub_speed_rpm);
  }

  void SetSteerAngle(float angle)
  {
    steer_angle_ = float((int(angle) % kMaxRotation) + homing_offset_angle_);
    units::angle::degree_t steer_angle_degree(steer_angle_);
    steer_motor_.SetAngle(steer_angle_degree, kSteerSpeed);
  };

  int GetHomingOffset() const
  {
    return homing_offset_angle_;
  }

  /// Checks if the steer wheel is aligned with slip ring
  bool IsHomed()
  {
    // homing_pin_.GetPin().settings.Floating();
    // if (homing_pin_.Read() == kHomeLevel) // w/ slip ring
    if (GetSteerAngle() == 0)  // no slip ring - for testing purposes
    {
      homing_offset_angle_ = int(steer_angle_);
      return true;
    }
    return false;
  }

 private:
  std::string_view name_ ;
  int homing_offset_angle_ = 0;
  float steer_angle_       = 0;
  float hub_speed_         = 0;

  sjsu::RmdX & hub_motor_;
  sjsu::RmdX & steer_motor_;
  sjsu::Gpio & homing_pin_;

  const bool kHomeLevel  = sjsu::Gpio::kHigh;
  const int kMaxRotation = 360;
  const float kMaxSpeed  = 100;
  const float kZero      = 0;
  const units::angular_velocity::revolutions_per_minute_t kSteerSpeed = 10_rpm;
};
}  // namespace sjsu::drive
