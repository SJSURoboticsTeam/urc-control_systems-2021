#pragma once

#include "arm_joint.hpp"

namespace sjsu::arm
{
class HumanArm
{
 public:
  struct MissionControlArmData
  {
    enum class ArmModes : char
    {
      kConcurrent = 'C',
      kRotunda    = 'R',
      kShoulder   = 'S',
      kElbow      = 'E',
      kHand       = 'D'
    };
    ArmModes arm_mode = ArmModes::kConcurrent;

    struct ArmAngles
    {
      int rotunda  = 0;
      int shoulder = 0;
      int elbow    = 0;
    };
    ArmAngles arm_angles;
  };

  HumanArm(ArmJoint & rotunda, ArmJoint & shoulder, ArmJoint & elbow)
      : rotunda_(rotunda), shoulder_(shoulder), elbow_(elbow){};

  void Initialize()
  {
    rotunda_.Initialize();
    shoulder_.Initialize();
    elbow_.Initialize();
  }

  void PrintArmData()
  {
    printf("JOINTS-DATA:\n");
    printf("=========================================\n");
    printf("Rotunda speed: %d\n", rotunda_.GetSpeed());
    printf("Rotunda position: %d\n", rotunda_.GetPosition());

    printf("Shoulder speed: %d\n", shoulder_.GetSpeed());
    printf("Shoulder position: %d\n", shoulder_.GetPosition());

    printf("Elbow speed: %d\n", elbow_.GetSpeed());
    printf("Elbow position: %d\n", elbow_.GetPosition());
  }

  void HomeArm(float speed)
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
    else if (ShoulderIsInThirdQuadrantOfGraph())
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

  void HandleConcurrentMode(float rotunda_angle,
                            float shoulder_angle,
                            float elbow_angle,
                            float speed)
  {
    MoveRotunda(rotunda_angle, speed);
    MoveShoulder(shoulder_angle, speed);
    MoveElbow(elbow_angle, speed);
  }


  void HandleMovement(float rotunda, float shoulder, float elbow, float speed)
  { 
      switch (current_arm_mode_)
    {
      case MissionControlArmData::ArmModes::kConcurrent:
        HandleConcurrentMode(rotunda, shoulder, elbow, speed);
        break;
      case MissionControlArmData::ArmModes::kRotunda:
        MoveRotunda(rotunda, speed);
        break;
      case MissionControlArmData::ArmModes::kShoulder:
        MoveShoulder(shoulder, speed);
        break;
      case MissionControlArmData::ArmModes::kElbow:
        MoveElbow(elbow, speed);
        break;
      case MissionControlArmData::ArmModes::kHand:
        break;
    }
  }

  MissionControlArmData::ArmModes GetCurrentArmMode()
  {
    return current_arm_mode_;
  }

  void SetCurrentArmMode(MissionControlArmData::ArmModes current_arm_mode)
  {
    current_arm_mode_ = current_arm_mode;
  }

  int GetRotundaPosition()
  {
    return rotunda_.GetPosition();
  }

  int GetRotundaSpeed()
  {
    return rotunda_.GetSpeed();
  }

  int GetShoulderPosition()
  {
    return shoulder_.GetPosition();
  }

  int GetShoulderSpeed()
  {
    return shoulder_.GetSpeed();
  }

  int GetElbowPosition()
  {
    return elbow_.GetPosition();
  }

  int GetElbowSpeed()
  {
    return elbow_.GetSpeed();
  }

  int GetRotundaOffsetAngle()
  {
    return rotunda_.GetOffsetAngle();
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

  bool ShoulderIsInThirdQuadrantOfGraph()
  {
    if (elbow_.acceleration_.x + rotunda_.acceleration_.x >= 0 &&
        elbow_.acceleration_.y + rotunda_.acceleration_.y >= 0)
    {
      return true;
    }
    return false;
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

  void UpdateAccelerations()
  {
    rotunda_.GetAccelerometerData();
    shoulder_.GetAccelerometerData();
    elbow_.GetAccelerometerData();
  }

  ArmJoint & rotunda_;
  ArmJoint & shoulder_;
  ArmJoint & elbow_;

  MissionControlArmData::ArmModes current_arm_mode_ =
      MissionControlArmData::ArmModes::kConcurrent;
};
}  // namespace sjsu::arm