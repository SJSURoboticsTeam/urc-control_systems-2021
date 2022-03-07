#pragma once

#include "utility/math/map.hpp"

namespace sjsu::arm
{
class Finger
{
 public:
  Finger(int pwm_pin) : pwm_pin_(pwm_pin) {}

  void SetPositionAndPwm(float angle)
  {
    position_ = std::clamp(angle, min_angle_, max_angle_);
    MapAngleToPWM(position_);
  }

  void SetSpeed(float target_speed)
  {
    speed_ = (target_speed);
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

  uint8_t GetPwmPin() const
  {
    return static_cast<uint8_t>(pwm_pin_);
  }

  units::time::microsecond_t GetPWM()
  {
    return pwm_;
  }

 private:
  void MapAngleToPWM(float angle)
  {
    float angle_to_pwm =
        static_cast<float>(sjsu::Map(angle, 0.0, 180.0, 1.0, 2.0));
    pwm_ = units::time::microsecond_t{ angle_to_pwm };
  }

  int pwm_pin_;

  units::time::microsecond_t pwm_ = 0_us;
  float position_                 = 0;
  float speed_                    = 0;
  float max_angle_                = 180;
  float min_angle_                = 0;
};

}  // namespace sjsu::arm