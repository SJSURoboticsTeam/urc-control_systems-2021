#pragma once
#include "utility/units.hpp"
#include "L2_HAL/actuators/servo/rmd_x.hpp"
#include "L2_HAL/sensors/movement/accelerometer/mpu6050.hpp"
#include "L1_Peripheral/lpc40xx/can.hpp"

namespace sjsu::arm
{
// the joint class is used for the rotunda, elbow, and shoulder motors.
class joint
{
 private:
  // The minimum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t minimum_angle;
  // The maximum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t maximum_angle;
  // The angle the joint will move to when is not operational
  units::angle::degree_t rest_angle;
  // The angle between the motor's zero position and the actual homed zero
  // positon.
  units::angle::degree_t zero_offset_angle;
  // Motor object that controls the joint
  sjsu::RmdX * motor;
  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 * mpu;

 public:
  joint();
  joint(units::angle::degree_t min_angle,
        units::angle::degree_t max_angle,
        units::angle::degree_t rest_angle,
        sjsu::RmdX joint_motor,
        sjsu::Mpu6050 accelerometer);
  void set_zero_offset(units::angle::degree_t offset);
  sjsu::Accelerometer::Acceleration_t get_accelerometer_data();
};
joint::joint()
{
  minimum_angle     = 0_deg;
  maximum_angle     = 180_deg;
  rest_angle        = 0_deg;
  zero_offset_angle = 0_deg;
}
}  // namespace sjsu::arm
