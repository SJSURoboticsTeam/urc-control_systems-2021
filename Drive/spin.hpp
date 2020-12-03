/*
  Contains the spin mode handler. 
  Early prototype - WIP
*/

#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

void spinMode(float speed, units::angle::degree_t angle)
{
  sjsu::LogInfo("Servo application starting...");

  // Creating PWM on pin 2.0
  sjsu::lpc40xx::Pwm pwm1(sjsu::lpc40xx::Pwm::Channel::kPwm0);
  sjsu::lpc40xx::Pwm pwm2(sjsu::lpc40xx::Pwm::Channel::kPwm1);
  sjsu::lpc40xx::Pwm pwm3(sjsu::lpc40xx::Pwm::Channel::kPwm2);

  // Create servo class and and give it the PWM.
  sjsu::Servo servo1(pwm1);
  sjsu::Servo servo2(pwm2);
  sjsu::Servo servo3(pwm3);

  sjsu::LogInfo("Initalizing Servo");
  servo1.Initialize();
  servo2.Initialize();
  servo3.Initialize();
  // When all of the bounds of the servo class are set, the servo class will
  // map your degrees to microseconds. With the below example, 0 degrees will
  // represent 500 us, 180 will represent 2500 us, and by linear correlation
  // 90 degrees will representation 1500 us.

  // Set RC servo PWM frequency to default (50 Hz)
  servo1.ConfigureFrequency();
  servo2.ConfigureFrequency();
  servo3.ConfigureFrequency();

  // Set the angle bounds of the servo to be 0 degrees and 180 degrees
  sjsu::LogInfo("Setting Servo angle bounds from 0 deg to 180 deg.");
  servo1.ConfigureAngleBounds(0_deg, 180_deg);
  servo2.ConfigureAngleBounds(0_deg, 180_deg);
  servo3.ConfigureAngleBounds(0_deg, 180_deg);

  sjsu::LogInfo("Enabling Servo!");
  servo1.Enable();
  servo2.Enable();
  servo3.Enable();

  servo1.SetAngle(90_deg);
  servo2.SetAngle(90_deg);
  servo3.SetAngle(90_deg);
  sjsu::Delay(5s);

  while (true)
  {
    // Command the servo to go to 180 degrees as fast as the servo will allow.
    servo1.SetAngle(180_deg);
    servo2.SetAngle(180_deg);
    servo3.SetAngle(180_deg);
    sjsu::Delay(10s);

    // Command the servo to go to 0 degrees as fast as the servo will allow.
    servo1.SetAngle(0_deg);
    servo2.SetAngle(0_deg);
    servo3.SetAngle(0_deg);
    sjsu::Delay(10s);
  }
  return 0;
}