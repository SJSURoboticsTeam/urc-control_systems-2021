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
    homing_offset_angle_ = 0;
    hub_motor_.Initialize();
    steer_motor_.Initialize();
    // homing_pin_.GetPin().settings.Floating();  // testing with slip ring
    homing_pin_.GetPin().settings.PullDown();  // testing with button
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  void Print()
  {
    printf("%-10s%-10d%-10d\n", name_.c_str(), GetHubSpeed(), GetSteerAngle());
  }

  std::string GetName()
  {
    return name_;
  }

  int GetHubSpeed()
  {
    return int(hub_speed_);
  };

  int GetSteerAngle()
  {
    return int(steer_angle_);
  };

  void SetHubSpeed(double speed)
  {
    hub_speed_ = float(std::clamp(speed, -kMaxSpeed, kMaxSpeed));
    units::angular_velocity::revolutions_per_minute_t hub_speed_rpm(hub_speed_);
    hub_motor_.SetSpeed(hub_speed_rpm);
  }

  void SetSteerAngle(double angle)
  {
    steer_angle_ = float((int(angle) % kMaxRotation) + homing_offset_angle_);
    units::angle::degree_t steer_angle_degree(steer_angle_);
    steer_motor_.SetAngle(steer_angle_degree, kSteerSpeed);
  };

  /// TESTING - Moves wheel to start position - press SJ2 button to exit
  void HomeWheel()
  {
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    sjsu::Button homing_button(homing_pin_);
    homing_button.Initialize();

    for (int angle = 0; angle < 360; angle += 2)
    {
      SetSteerAngle(0);
      sjsu::Delay(50ms);
      if (homing_pin_.Read() == kHomeLevel)
      {
        homing_offset_angle_ = 0;
        break;
      }
    }
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring
  void SlipRingHomeWheel()
  {
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());

    for (int angle = 0; angle < 360; angle += 2)
    {
      SetSteeringAngle(angle);
      if(!(Uptime() % 50))
      {
        if (homing_pin_.Read() == home_level)
        {
          homing_offset_angle_ = angle;
          break;
        }
      }
      SetSteerAngle(angle);
      sjsu::Delay(50ms);
    }
    sjsu::LogInfo("%s wheel offset: %d", name_.c_str(), homing_offset_angle_);
  };

  bool IsWheelHomed()
  {
      if (homing_pin_.Read() == kHomeLevel)
        {
          homing_offset_angle = steer_angle_;
          sjsu::LogInfo("Homing %s wheel done! Offset angle set to %d", name_.c_str(),
                            homing_offset_angle_.to<int>());
          return true;
        }
    return false
  }

 private:
  std::string name_        = "";
  int homing_offset_angle_ = 0;
  float steer_angle_       = 0;
  float hub_speed_         = 0;

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
