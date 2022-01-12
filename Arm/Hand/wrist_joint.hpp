#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
class WristJoint
{
 public:
   struct Acceleration
  {
    float x=0;
    float y=0;
    float z=0;
  };
  
  WristJoint(sjsu::RmdX & left_joint_motor,
             sjsu::RmdX & right_joint_motor,
             sjsu::Mpu6050 & accelerometer)
      : left_motor_(left_joint_motor),
        right_motor_(right_joint_motor),
        mpu_(accelerometer)
  {
  }

  void Initialize()
  {
    left_motor_.Initialize();
    right_motor_.Initialize();
    mpu_.Initialize();
  }

  void SetPitchPosition(float pitch_angle)
  {
    pitch_angle_ = float(std::clamp(pitch_angle, kPitchMinimumAngle, kPitchMaximumAngle));
    units::angle::degree_t angle_to_degrees(pitch_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

    void SetRollPosition(float roll_angle)
  {
    roll_angle_ = float(std::clamp(roll_angle, kRollMinimumAngle, kRollMaximumAngle));
    units::angle::degree_t angle_to_degrees(roll_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffsets(float left_offset,
                      float right_offset)
  {
    left_offset_angle_  = left_offset;
    right_offset_angle_ = right_offset;
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  Acceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(mpu_.Read());
    Acceleration acceleration;
    acceleration.x = static_cast<float>(acceleration_to_float.x);
    acceleration.y = static_cast<float>(acceleration_to_float.y);
    acceleration.z = static_cast<float>(acceleration_to_float.z);
    return acceleration;
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

  sjsu::RmdX & left_motor_;
  sjsu::RmdX & right_motor_;
  sjsu::Mpu6050 & mpu_;

  float pitch_angle_        = 0;
  float roll_angle_         = 0;
  float left_offset_angle_  = 0;
  float right_offset_angle_ = 0;

  const float kPitchMinimumAngle = 0;
  const float kPitchMaximumAngle = 180;
  const float kPitchRestAngle    = 90;
  const float kRollMinimumAngle  = 0;
  const float kRollMaximumAngle  = 180;
  const float kRollRestAngle     = 90;
};
}  // namespace sjsu::arm
