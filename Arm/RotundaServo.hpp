/**
 * Header file for rotunda servo motor functionality.
 *
 * @author SJSU Control Systems
 * @version Spring 2020
 */

#pragma once
#include "L2_HAL/actuators/servo/servo.hpp"

namespace sjsu
{
namespace robotics
{
class RotundaServo : public Servo
{
 public:
  /**
   * Contructs RotundaServo object. Takes in 1 pwm pin parameter.
   *
   * @param pwm The pin to control the RotundaServo object.
   */
  explicit constexpr RotundaServo(const Pwm & pwm) : Servo(pwm) {}

  /**
   * Initializes frequency, pulse, and angle bounds of RotundaServo object.
   * 
   * @param frequency Defaulted to 50_Hz for servo motors. 
   */
  void Initialize(units::frequency::hertz_t frequency = 50_Hz) override
  {
    Servo::Initialize(frequency);
    Servo::SetAngleBounds(-1540_deg, 1540_deg);
    Servo::SetPulseBounds(500us, 2500us);
  }
};
}  // namespace robotics
}  // namespace sjsu
