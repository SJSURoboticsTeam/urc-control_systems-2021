#pragma once

#include "devices/actuators/servo/servo.hpp"
#include "Arm-Subsystem/Hand/pca9685.hpp"
namespace sjsu::arm
{
class Finger
{
 public:
  Finger(sjsu::Servo & servo) : servo_(servo) {}
  Finger()
  Finger(sjsu::Servo & servo,
         int pin_number,
         float min_pulse,
         float max_pulse,
         float position,
         float speed,
         float max_angle,
         float min_angle)
      : servo_(servo),
        pin_number_(pin_number),
        position_(position),
        speed_(speed),
        max_angle_(max_angle),
        min_angle_(min_angle),
        min_pulse_(min_pulse),
        max_pulse_(max_pulse),
  {
  }

  void Initialize()
  {
    servo_.ModuleInitialize();
  }


  void SetPosition(float angle)
  {
    position_ = sjsu:Map(angle, min_angle_, max_angle_, min_pulse_, max_pulse_);
    units::chrono::milliseconds position_(position_);
    // units::angle::degree_t angle_to_degrees(position_);
    // servo_.SetAngle(angle_to_degrees);
    pca.setPulseWidth(pin_number_,position_);
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

 private:
  sjsu::Servo & servo_;

  float position_  = 0;
  float speed_     = 0;
  float max_angle_ = 180;
  float min_angle_ = 0;
  float min_pulse_ = 2.0;
  float max_pulse_ = 1.0;
  int pin_number_ = 0;
};

}  // namespace sjsu::arm