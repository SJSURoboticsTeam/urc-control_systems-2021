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
    float x;
    float y;
    float z;
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
    pitch_angle_ = pitch_angle;
    units::angle::degree_t angle_to_degrees(pitch_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  // Move the wrist to its callibrated roll and pitch angles
  void SetPosition(float pitch_angle,
                   float roll_angle)
  {
    //units::angle::degree_t pitch_angle_to_degree(pitch_angle);
    //units::angle::degree_t roll_angle_to_degree(roll_angle);
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffsets(float left_offset,
                      float right_offset)
  {
    units::angle::degree_t left_offset_to_degree(left_offset);
    units::angle::degree_t right_offset_to_degree(right_offset);
<<<<<<< HEAD
    left_zero_offset_angle  = left_offset;
    right_zero_offset_angle = right_offset;
=======
    left_zero_offset_angle  = left_offset_to_degree;
    right_zero_offset_angle = right_offset_to_degree;
>>>>>>> 3a0311f00ca2dabf3f6d7b6c602d9b2b2231005c
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  Acceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(mpu.Read());
    Acceleration acceleration;
    acceleration.x = static_cast<float>(acceleration_to_float.x);
    acceleration.y = static_cast<float>(acceleration_to_float.y);
    acceleration.z = static_cast<float>(acceleration_to_float.z);
    return acceleration;
  }

int GetRollPosition(){
  return 0;
}
private:
<<<<<<< HEAD
  struct Acceleration
  {
    float x;
    float y;
    float z;
  }
=======
>>>>>>> 3a0311f00ca2dabf3f6d7b6c602d9b2b2231005c

  const float kPitchMinimumAngle = 0;
  const float kPitchMaximumAngle = 180;
  const float kPitchRestAngle    = 90;
  const float kRollMinimumAngle  = 0;
  const float kRollMaximumAngle  = 180;
  const float kRollRestAngle     = 90;
};
}  // namespace sjsu::arm
