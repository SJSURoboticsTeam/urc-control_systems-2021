#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
class WristJoint
{
 public:
  WristJoint(sjsu::RmdX & left_joint_motor,
             sjsu::RmdX & right_joint_motor,
             sjsu::Mpu6050 & accelerometer)
      : left_motor_(left_joint_motor),
        right_motor_(right_joint_motor),
        mpu_(accelerometer)
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
      : pitch_minimum_angle_(pitch_min_angle),
        pitch_maximum_angle_(pitch_max_angle),
        pitch_rest_angle_(pitch_standby_angle),
        roll_minimum_angle_(roll_min_angle),
        roll_maximum_angle_(roll_max_angle),
        roll_rest_angle_(roll_standby_angle),
        left_motor_(left_joint_motor),
        right_motor_(right_joint_motor),
        mpu_(accelerometer)
  {
  }

  /// Initialize the WristJoint object, This must be called before any other
  /// function.
  void Initialize()
  {
    left_motor_.Initialize();
    right_motor_.Initialize();
    mpu_.Initialize();
  }

  // Move the wrist to its callibrated roll and pitch angles
  void SetPitchPosition(double pitch_angle)
  {
    units::angle::degree_t angle_to_degrees(pitch_angle);
    pitch_angle_ = angle_to_degrees;
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  void SetRollPoistion(double roll_angle)
  {
    units::angle::degree_t angle_to_degrees(roll_angle);
    roll_angle_ = angle_to_degrees;
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffsets(units::angle::degree_t left_offset,
                      units::angle::degree_t right_offset)
  {
    left_zero_offset_angle_  = left_offset;
    right_zero_offset_angle_ = right_offset;
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  sjsu::Accelerometer::Acceleration_t GetAccelerometerData()
  {
    return mpu_.Read();
  }
  int GetPitchPosition()
  {
    return int(pitch_angle_);
  }

  int GetRollPosition()
  {
    return int(roll_angle_);
  }

 private:
  // The Minimum and Maximum angles that the wrist pitch is allowed to rotate.
  units::angle::degree_t pitch_minimum_angle_ = 0_deg;
  units::angle::degree_t pitch_maximum_angle_ = 180_deg;
  // The pitch angle of the wrist when not in operation.
  units::angle::degree_t pitch_rest_angle_ = 90_deg;
  // The Minimum and Maximum angles that the wrist roll is allowed to rotate.
  units::angle::degree_t roll_minimum_angle_ = 0_deg;
  units::angle::degree_t roll_maximum_angle_ = 180_deg;
  // The roll angle of the wrist when not in operation.
  units::angle::degree_t roll_rest_angle_ = 90_deg;

  // angle of the pitch and roll of the wrist
  units::angle::degree_t pitch_angle_ = 0_deg;
  units::angle::degree_t roll_angle_  = 0_deg;

  // The wrist uses two Rmd_x7 motors in a differential drive to control its
  // pitch and roll. If the motors are traveling in the same speed and
  // direction, then the pitch of the wrist will change, a difference in speed
  // causes roll

  // The angle between the motors' zero positon and the actual homed zero
  // positions.
  units::angle::degree_t left_zero_offset_angle_  = 0_deg;
  units::angle::degree_t right_zero_offset_angle_ = 0_deg;
  sjsu::RmdX & left_motor_;
  sjsu::RmdX & right_motor_;

  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 & mpu_;
};
}  // namespace sjsu::arm
