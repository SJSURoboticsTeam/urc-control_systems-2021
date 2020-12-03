/**
 * Header file for drive control functionality.
 *
 * @author SJSU Robotics - Control Systems
 * @version Spring 2020
 */

#pragma once

#include "L3_Application/Robotics/Drive/HubMotor.hpp"
#include "L3_Application/Robotics/Common/ChiHaiServo.hpp"

namespace sjsu
{
namespace robotics
{
class DriveController
{
 public:
  /**
   * Contructs DriveController object. Takes in 3 HubMotor and 3 ChiHaiServo
   * objects as parameters. The HubMotor are the motors controlling the speed
   * and direction of the wheel. The ChiHaiServo are the motors controlling the
   * angle of the wheels.
   *
   * @param wheel_X the three hub motors used spinning the wheels.
   * @param steering_X the three steering motors used for direction.
   */
  explicit constexpr DriveController(sjsu::robotics::HubMotor & wheel_a,
                                     sjsu::robotics::HubMotor & wheel_b,
                                     sjsu::robotics::HubMotor & wheel_c,
                                     sjsu::robotics::ChiHaiServo & steering_a,
                                     sjsu::robotics::ChiHaiServo & steering_b,
                                     sjsu::robotics::ChiHaiServo & steering_c)
      : wheel_a_(wheel_a),
        wheel_b_(wheel_b),
        wheel_c_(wheel_c),
        steering_a_(steering_a),
        steering_b_(steering_b),
        steering_c_(steering_c)
  {
  }

  /**
   * Initializes DriveController object.
   */
  void Initialize()
  {
    wheel_a_.Initialize();
    wheel_b_.Initialize();
    wheel_c_.Initialize();
    steering_a_.Initialize();
    steering_b_.Initialize();
    steering_c_.Initialize();
  }

  /**
   * Stops all motors associated with driveController.
   */
  void Stop()
  {
    steering_a_.Stop();
    steering_b_.Stop();
    steering_c_.Stop();
    wheel_a_.Stop();
    wheel_b_.Stop();
    wheel_c_.Stop();
  }

  /**
   * Assigns speed and direction of wheel A.
   *
   * @param duty_cycle Speed of the wheel from 0.0 < x < 1.0 float.
   * @param direction assigns forward or reverse direction of wheel.
   */
  void SetWheelASpeedAndDirection(float duty_cycle,
                                  sjsu::robotics::HubMotor::Direction direction)
  {
    wheel_a_.SetSpeedAndDirection(duty_cycle, direction);
  }

  /**
   * Assigns speed and direction of wheel B.
   *
   * @param duty_cycle Speed of the wheel from 0.0 < x < 1.0 float.
   * @param direction assigns forward or reverse direction of wheel.
   */
  void SetWheelBSpeedAndDirection(float duty_cycle,
                                  sjsu::robotics::HubMotor::Direction direction)
  {
    wheel_a_.SetSpeedAndDirection(duty_cycle, direction);
  }

  /**
   * Assigns speed and direction of wheel C.
   *
   * @param duty_cycle Speed of the wheel from 0.0 < x < 1.0 float.
   * @param direction assigns forward or reverse direction of wheel.
   */
  void SetWheelCSpeedAndDirection(float duty_cycle,
                                  sjsu::robotics::HubMotor::Direction direction)
  {
    wheel_a_.SetSpeedAndDirection(duty_cycle, direction);
  }

  /**
   * Assigns a new angle to wheel A.
   *
   * @param angle The degrees to turn the wheel. (X_deg)
   */
  void SetWheelAAngle(units::angle::degree_t angle)
  {
    steering_a_.SetAngle(angle);
  }

  /**
   * Assigns a new angle to wheel B.
   *
   * @param angle The degrees to turn the wheel. (X_deg)
   */
  void SetWheelBAngle(units::angle::degree_t angle)
  {
    steering_b_.SetAngle(angle);
  }

  /**
   * Assigns a new angle to wheel C.
   *
   * @param angle The degrees to turn the wheel. (X_deg)
   */
  void SetWheelCAngle(units::angle::degree_t angle)
  {
    steering_c_.SetAngle(angle);
  }

 private:
  sjsu::robotics::HubMotor & wheel_a_;
  sjsu::robotics::HubMotor & wheel_b_;
  sjsu::robotics::HubMotor & wheel_c_;
  sjsu::robotics::ChiHaiServo & steering_a_;
  sjsu::robotics::ChiHaiServo & steering_b_;
  sjsu::robotics::ChiHaiServo & steering_c_;
};
}  // namespace robotics
}  // namespace sjsu
