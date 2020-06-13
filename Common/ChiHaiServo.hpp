/**
 * Header file for Chi Hai DC Brushless motor functionality.
 *
 * @author SJSU Robotics - Control Systems
 * @version Spring 2020
 */

#pragma once

#include "L3_Application/Robotics/Common/MagneticEncoder.hpp"
#include "L3_Application/Robotics/Common/Drv8801.hpp"
#include "utility/units.hpp"

namespace sjsu
{
namespace robotics
{
class ChiHaiServo
{
 private:
  static constexpr float kResolution = 0.001f;

  /**
   * Checks whether the ChiHaiServo has moved within respectable degree of
   * desired angle. Maybe uncomplete.
   *
   * @param expected degree to move towards.
   * @param actual current degree of ChiHaiServo.
   * @param resolution acceptable difference from expected and actual angles.
   * @return whether the current angle is close enough to the desired angle.
   */
  bool ApproximatelyEqual(units::angle::degree_t expected,
                          units::angle::degree_t actual,
                          float resolution)
  {
    return (-resolution < (actual.to<float>() - expected.to<float>())) &&
           ((actual.to<float>() - expected.to<float>()) < resolution);
  }

 public:
  /**
   * Constructs a ChiHaiServo object. Contains a MagneticEncoder and Drv8801.
   */
  explicit constexpr ChiHaiServo(
      sjsu::robotics::MagneticEncoder magnetic_encoder,
      sjsu::robotics::Drv8801 drv)
      : magnetic_encoder_(magnetic_encoder), drv_(drv)
  {
  }

  /**
   * Initializes a ChaiHaiServo object.
   */
  void Initialize()
  {
    magnetic_encoder_.Initialize();
    drv_.Initialize();
  }

  /**
   * Sets the angle of the Chi Hai motor by angle. Used by a PID controller.
   *
   * @see PID controller
   * @param angle The degrees to move the Chi Hai motor.
   */
  void SetAngle(units::angle::degree_t angle)
  {
    units::angle::degree_t current_degree = magnetic_encoder_.GetAngle();
    if (angle - current_degree > 0_deg)
    {
      drv_.TurnForward;
    }
    else
    {
      drv_.TurnBackward;
    }
    // while (!ApproximatelyEqual(current_degree, angle, kResolution))
    // {
    //   drv_.TurnBackward();
    // }
    drv_.Stop();
  }

/**
 * Instantly stops ChiHai motor movement.
 * 
 * @see exponential moving average
 */
  void Stop()
  {
    drv_.Stop();
  }

 private:
  sjsu::robotics::MagneticEncoder magnetic_encoder_;
  sjsu::robotics::Drv8801 drv_;
};
}  // namespace robotics
}  // namespace sjsu
