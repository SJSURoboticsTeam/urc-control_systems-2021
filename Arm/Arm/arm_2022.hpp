#pragma once
#include "arm.hpp"
#include "arm_joint.hpp"

namespace sjsu::arm
{
class Arm2022 : public Arm
{
  void Initalize() override 
  {
      rotunda_.Initialize();
      shoulder_.Initialize();
      elbow_.Initialize();
  }

  void UpdateAccelerations()
  {
    rotunda_.GetAccelerometerData();
    shoulder_.GetAccelerometerData();
    elbow_.GetAccelerometerData();
  }

  void HomeArm(float speed) override
  {
    HomeShoulder(speed);
    HomeElbow(speed);
  }

  void HomeShoulder(float speed)
  {
    UpdateAccelerations();
    float home_angle = 0;

    home_angle = CalculateShoulderHomeAngle();
    shoulder_.SetZeroOffset(home_angle);
    MoveShoulder(home_angle, speed);
  }

  void HomeElbow(float speed)
  {
    UpdateAccelerations();
    float home_angle = 0;

    float angle_without_correction = CalculateUncorrectedElbowHomeAngle();

    if (ShoulderIsInSecondQuadrantOfGraph())
    {
      home_angle = 90 - angle_without_correction;
    }
    else if (ShoulderIsInThirdQuandrantOfGraph())
    {
      home_angle = 180 + angle_without_correction;
    }
    else
    {
      home_angle = angle_without_correction;
    }
    elbow_.SetZeroOffset(home_angle);
    MoveElbow(home_angle, speed);
  }

  void MoveRotunda(float angle, float speed)
  {
    rotunda_.SetJointSpeed(speed);
    rotunda_.SetPosition(angle);
  }

  void MoveShoulder(float angle, float speed)
  {
    shoulder_.SetJointSpeed(speed);
    shoulder_.SetPosition(angle);
  }

  void MoveElbow(float angle, float speed)
  {
    elbow_.SetJointSpeed(speed);
    elbow_.SetPosition(angle);
  }

  void HandleConcurrentMode(float rotunda_angle, float shoulder_angle, float elbow_angle)
  {
    MoveRotunda(rotunda_angle);
    MoveShoulder(shoulder_angle);
    MoveElbow(elbow_angle);
  }

  void HandleMovement(current_arm_mode_) override
  {
    switch (current_arm_mode_)
    {
      case MissionControlData::ArmModes::kHomeArm: HomeArm(); break;
      case MissionControlData::ArmModes::kConcurrent:
        HandleConcurrentMode();
        break;
      case MissionControlData::ArmModes::kRotunda:
        MoveRotunda(mc_data_.rotunda_angle);
        break;
      case MissionControlData::ArmModes::kShoulder:
        MoveShoulder(mc_data_.shoulder_angle);
        break;
      case MissionControlData::ArmModes::kElbow:
        MoveElbow(mc_data_.elbow_angle);
        break;
    }
  }


  private:

  float CalculateShoulderHomeAngle()
  {
    float home_angle = 0;

    float acceleration_x = rotunda_.acceleration_.x + shoulder_.acceleration_.x;
    float acceleration_y = rotunda_.acceleration_.y + shoulder_.acceleration_.y;

    home_angle = float(atan(acceleration_y / acceleration_x));
    return home_angle;
  }

  float CalculateUncorrectedElbowHomeAngle()
  {
    float acceleration_x = rotunda_.acceleration_.x + elbow_.acceleration_.x;
    float acceleration_y = rotunda_.acceleration_.y + elbow_.acceleration_.y;
    float angle_without_correction =
        float(atan(acceleration_y / acceleration_x));
    return angle_without_correction;
  }

  bool ShoulderIsInSecondQuadrantOfGraph()
  {
    if (elbow_.acceleration_.x + rotunda_.acceleration_.x >= 0 &&
        elbow_.acceleration_.y + rotunda_.acceleration_.y <= 0)
    {
      return true;
    }
    return false;
  }

  bool ShoulderIsInThirdQuandrantOfGraph()
  {
    if (elbow_.acceleration_.x + rotunda_.acceleration_.x >= 0 &&
        elbow_.acceleration_.y + rotunda_.acceleration_.y >= 0)
    {
      return true;
    }
    return false;
  }

  ArmJoint rotunda_;
  ArmJoint shoulder_;
  ArmJoint elbow_;

};
} //namespace sjsu::arm