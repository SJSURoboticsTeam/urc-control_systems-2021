#pragma once

#include "devices/actuators/servo/servo.hpp"

namespace sjsu::arm
{
class Finger
{
 public:
  Finger(sjsu::Servo & servo) : servo_(servo) {}

  Finger(sjsu::Servo & servo,
         float position,
         float speed,
         float max_angle,
         float min_angle,
         int pwm_pin)
      : servo_(servo),
        position_(position),
        speed_(speed),
        max_angle_(max_angle),
        min_angle_(min_angle),
        pwm_pin_(pwm_pin)
  {
  }

  void Initialize()
  {
    servo_.ModuleInitialize();
  }

  void SetPosition(float angle)
  {
    position_ = std::clamp(angle, min_angle_, max_angle_);
    units::angle::degree_t angle_to_degrees(position_);
    servo_.SetAngle(angle_to_degrees);
  }

  void Home()
  {
    return;
  }

  void SetSpeed(float target_speed)
  {
    speed_ = target_speed;
  }

  int GetPosition() const
  {
    return int(position_);
  }

  int GetSpeed() const
  {
    return int(speed_);
  }

  int GetMaxAngle() const
  {
    return int(max_angle_);
  }

  int GetMinAngle() const
  {
    return int(min_angle_);
  }

  int GetPwmPin() const
  {
    return pwm_pin_;
  }

 private:
  sjsu::Servo & servo_;
  int pwm_pin_;

  float position_  = 0;
  float speed_     = 0;
  float max_angle_ = 180;
  float min_angle_ = 0;
};

}  // namespace sjsu::arm