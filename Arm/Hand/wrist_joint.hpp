#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "joint.hpp"

namespace sjsu::arm
{
class WristJoint : public Joint
{
 public:
  WristJoint(sjsu::RmdX & left_joint_motor,
             sjsu::RmdX & right_joint_motor,
             sjsu::Mpu6050 & accelerometer)
      : Joint(accelerometer),
        left_motor_(left_joint_motor),
        right_motor_(right_joint_motor)
  {
  }

  void Initialize()
  {
    left_motor_.Initialize();
    right_motor_.Initialize();
    mpu_.Initialize();
  }

  // Sets Pitch Position of the wrist joint
  void SetPitchPosition(float pitch_angle)
  {
    pitch_angle_ = float(std::clamp(pitch_angle + pitch_offset_angle_,
                                    kPitchMinimumAngle, kPitchMaximumAngle));
    units::angle::degree_t angle_to_degrees(pitch_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroPitchOffsets(float pitch_offset)
  {
    pitch_offset_angle_ = pitch_offset;
  }

  // Sets Roll Position of the wrist joint
  void SetRollPosition(float roll_angle)
  {
    roll_angle_ = float(std::clamp(roll_angle + roll_offset_angle_,
                                   kRollMinimumAngle, kRollMaximumAngle));
    units::angle::degree_t angle_to_degrees(roll_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  void SetZeroRollOffsets(float roll_offset)
  {
    roll_offset_angle_ = roll_offset;
  }

  int GetPitchPosition()
  {
    return int(pitch_angle_);
  }

  int GetRollPosition()
  {
    return int(roll_angle_);
  }

  int GetPitchOffsetAngle()
  {
    return int(pitch_offset_angle_);
  }

  int GetRollOffsetAngle()
  {
    return int(roll_offset_angle_);
  }

 private:
  sjsu::RmdX & left_motor_;
  sjsu::RmdX & right_motor_;

  float pitch_angle_        = 0;
  float roll_angle_         = 0;
  float pitch_offset_angle_ = 0;
  float roll_offset_angle_  = 0;

  const float kPitchMinimumAngle = 0;
  const float kPitchMaximumAngle = 180;
  const float kPitchRestAngle    = 90;
  const float kRollMinimumAngle  = 0;
  const float kRollMaximumAngle  = 180;
  const float kRollRestAngle     = 90;
};
}  // namespace sjsu::arm
