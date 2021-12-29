#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
class WristJoint
{
 private:
  // The Minimum and Maximum angles that the wrist pitch is allowed to rotate.
  units::angle::degree_t pitch_minimum_angle = 0_deg;
  units::angle::degree_t pitch_maximum_angle = 180_deg;
  // The pitch angle of the wrist when not in operation.
  units::angle::degree_t pitch_rest_angle = 90_deg;
  // The Minimum and Maximum angles that the wrist roll is allowed to rotate.
  units::angle::degree_t roll_minimum_angle = 0_deg;
  units::angle::degree_t roll_maximum_angle = 180_deg;
  // The roll angle of the wrist when not in operation.
  units::angle::degree_t roll_rest_angle = 90_deg;

  // The wrist uses two Rmd_x7 motors in a differential drive to control its
  // pitch and roll. If the motors are traveling in the same speed and
  // direction, then the pitch of the wrist will change, a difference in speed
  // causes roll

  // The angle between the motors' zero positon and the actual homed zero
  // positions.
  units::angle::degree_t left_zero_offset_angle  = 0_deg;
  units::angle::degree_t right_zero_offset_angle = 0_deg;
  sjsu::RmdX & left_motor;
  sjsu::RmdX & right_motor;

  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 & mpu;

 public:
  WristJoint(sjsu::RmdX & left_joint_motor,
             sjsu::RmdX & right_joint_motor,
             sjsu::Mpu6050 & accelerometer)
      : left_motor(left_joint_motor),
        right_motor(right_joint_motor),
        mpu(accelerometer)
  {
  }

  WristJoint(sjsu::RmdX & left_joint_motor,
             sjsu::RmdX & right_joint_motor,
             sjsu::Mpu6050 & accelerometer,
             units::angle::degree_t pitch_min_angle,
             units::angle::degree_t pitch_max_angle,
             units::angle::degree_t pitch_standby_angle,
             units::angle::degree_t roll_min_angle,
             units::angle::degree_t roll_max_angle,
             units::angle::degree_t roll_standby_angle)
      : pitch_minimum_angle(pitch_min_angle),
        pitch_maximum_angle(pitch_max_angle),
        pitch_rest_angle(pitch_standby_angle),
        roll_minimum_angle(roll_min_angle),
        roll_maximum_angle(roll_max_angle),
        roll_rest_angle(roll_standby_angle),
        left_motor(left_joint_motor),
        right_motor(right_joint_motor),
        mpu(accelerometer)
  {
  }

  /// Initialize the WristJoint object, This must be called before any other
  /// function.
  void Initialize()
  {
    left_motor.Initialize();
    right_motor.Initialize();
    mpu.Initialize();
  }

  // Move the wrist to its callibrated roll and pitch angles
  void SetPosition(units::angle::degree_t pitch_angle,
                   units::angle::degree_t roll_angle)
  {
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffsets(units::angle::degree_t left_offset,
                      units::angle::degree_t right_offset)
  {
    left_zero_offset_angle  = left_offset;
    right_zero_offset_angle = right_offset;
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  sjsu::Accelerometer::Acceleration_t GetAccelerometerData()
  {
    return mpu.Read();
  }
};
}  // namespace sjsu::arm
