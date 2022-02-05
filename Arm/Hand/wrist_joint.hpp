#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "joint.hpp"
#include <cmath>

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
    Joint::Initialize();
  }
  void PrintWristData()
  {
    printf("Wrist Positions:\n");
    printf("Wrist pitch position: %d\n", pitch_angle_);
    printf("Wrist roll position: %d\n", roll_angle_);
  }

  // Sets Roll Position of the wrist joint
  void SetRollPosition(float speed, float roll_angle)
  {
    SetSpeed(speed);
    roll_angle_ = float(std::clamp(roll_angle + roll_offset_angle_,
                                       kRollMinimumAngle, kRollMaximumAngle));
    units::angle::degree_t angle_to_degrees(roll_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  // Sets Pitch Position of the wrist joint
  void SetPitchPosition(float speed, float pitch_angle)
  {
    SetSpeed(speed);
    pitch_angle_ = float(std::clamp(pitch_angle + pitch_offset_angle_,
                                    kPitchMinimumAngle, kPitchMaximumAngle));
    units::angle::degree_t angle_to_degrees(pitch_angle);
    left_motor_.SetAngle(angle_to_degrees);
    right_motor_.SetAngle(angle_to_degrees);
  }

  void HandleWristMovement(float speed, float roll, float pitch)
  {
    SetRollPosition(speed, roll);
    SetPitchPosition(speed, pitch);
  }

  /// Sets the zero_offset_angle value that the motors use to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroPitchOffsets(float pitch_offset)
  {
    pitch_offset_angle_ = pitch_offset;
  }

  void SetZeroRollOffsets(float roll_offset)
  {
    roll_offset_angle_ = roll_offset;
  }

  // Sets speed of the wrist joints
  void SetSpeed(float target_speed)
  {
    speed_ = std::clamp(target_speed, -kMaxSpeed, kMaxSpeed);
    units::angular_velocity::revolutions_per_minute_t speed_to_rpm(speed_);
    left_motor_.SetSpeed(speed_to_rpm);
    right_motor_.SetSpeed(speed_to_rpm);
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

  int GetSpeed()
  {
    return speed_;
  }

  void Home(float speed, float rotunda_offset)
  {
    GetAccelerometerData();
    HomePitch(speed, rotunda_offset);
    HomeRoll();
  };

  void HomePitch(float speed, float rotunda_offset)
  {
    float wrist_pitch_offset =
        float(atan(acceleration_.y / acceleration_.z)) + rotunda_offset;
    SetPitchPosition(speed, wrist_pitch_offset);
    SetZeroPitchOffsets(wrist_pitch_offset);
  }

  // can't home yet
  void HomeRoll(){};

 private:
  sjsu::RmdX & left_motor_;
  sjsu::RmdX & right_motor_;

  float pitch_angle_        = 0;
  float roll_angle_         = 0;
  float pitch_offset_angle_ = 0;
  float roll_offset_angle_  = 0;

  float speed_ = 0;

  const float kPitchMinimumAngle = 0;
  const float kPitchMaximumAngle = 180;
  const float kPitchRestAngle    = 90;
  const float kRollMinimumAngle  = 0;
  const float kRollMaximumAngle  = 180;
  const float kRollRestAngle     = 90;
  const float kMaxSpeed          = 100;
};
}  // namespace sjsu::arm
