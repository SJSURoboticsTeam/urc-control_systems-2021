#pragma once
#include "utility/units.hpp"
#include "L2_HAL/actuators/servo/rmd_x.hpp"
#include "L2_HAL/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
class wrist_joint
{
 private:
  // The Minimum and Maximum angles that the wrist pitch is allowed to rotate.
  units::angle::degree_t pitch_minimum_angle;
  units::angle::degree_t pitch_maximum_angle;
  // The pitch angle of the wrist when not in operation.
  units::angle::degree_t pitch_rest_angle;
  // The Minimum and Maximum angles that the wrist roll is allowed to rotate.
  units::angle::degree_t roll_minimum_angle;
  units::angle::degree_t roll_maximum_angle;
  // The roll angle of the wrist when not in operation.
  units::angle::degree_t roll_rest_angle;

  // The wrist uses two Rmd_x7 motors in a differential drive to control its
  // pitch and roll. If the motors are traveling in the same speed and
  // direction, then the pitch of the wrist will change, a difference in speed
  // causes roll

  // The angle between the motors' zero positon and the actual homed zero
  // positions.
  units::angle::degree_t left_zero_offset_angle;
  units::angle::degree_t right_zero_offset_angle;
  sjsu::RmdX * left_motor;
  sjsu::RmdX * right_motor;

  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 * mpu;

 public:
  wrist_joint()
  {
    pitch_minimum_angle     = 0_deg;
    pitch_maximum_angle     = 40_deg;
    pitch_rest_angle        = 20_deg;
    roll_minimum_angle      = 0_deg;
    roll_maximum_angle      = 180_deg;
    roll_rest_angle         = 90_deg;
    left_zero_offset_angle  = 0_deg;
    right_zero_offset_angle = 0_deg;
  }
};
}  // namespace sjsu::arm
