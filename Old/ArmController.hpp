/**
 * Header file for arm control functionality.
 *
 * @author SJSU Control Systems
 * @version Spring 2020
 */

#pragma once

#include "utility/units.hpp"
#include "L3_Application/Robotics/Common/ChiHaiServo.hpp"
#include "L3_Application/Robotics/Arm/RotundaServo.hpp"

namespace sjsu
{
namespace robotics
{
class ArmController
{
 public:
  /**
   * Contructs ArmController object. Takes in 3 servo objects as parameters.
   *
   * @param shoulder The ChiHaiServo object which controls the motor outside the
   * wrist.
   * @param wrist The ChiHaiServo object which controls the motor located within
   * the wrist.
   * @param rotunda The RotundaServo object which controls entire arm direction.
   * Located at arm base.
   */
  explicit constexpr ArmController(sjsu::robotics::ChiHaiServo & shoulder,
                                   sjsu::robotics::ChiHaiServo & wrist,
                                   sjsu::robotics::RotundaServo & rotunda)
      : shoulder_(shoulder), wrist_(wrist), rotunda_(rotunda)
  {
  }

  /**
   * Initializes ArmController object.
   */
  void Initialize()
  {
    shoulder_.Initialize();
    wrist_.Initialize();
    rotunda_.Initialize();
  }

  /**
   * Sets shoulder servo motor to desired angle.
   *
   * @param angle the number of degrees to move shoulder.
   */
  void SetShoulderAngle(units::angle::degree_t angle)
  {
    shoulder_.SetAngle(angle);
  }

  /**
   * Sets wrist servo motor to desired angle.
   *
   * @param angle the number of degrees to move wrist.
   */
  void SetWristAngle(units::angle::degree_t angle)
  {
    wrist_.SetAngle(angle);
  }

  /**
   * Sets rotunda servo motor to desired angle.
   *
   * @param angle the number of degrees to move rotunda.
   */
  void SetRotundaAngle(units::angle::degree_t angle)
  {
    rotunda_.SetAngle(angle);
  }

  /**
   * Stops the shoulder and wrist servo motors.
   */
  void Stop()
  {
    shoulder_.Stop();
    wrist_.Stop();
  }

 private:
  sjsu::robotics::ChiHaiServo & shoulder_;
  sjsu::robotics::ChiHaiServo & wrist_;
  sjsu::robotics::RotundaServo & rotunda_;
  /* data */
};

}  // namespace robotics
}  // namespace sjsu
// ChiHaiServo.hpp World
