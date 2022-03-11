#pragma once

#include "arm_joint.hpp"

namespace sjsu::arm
{
class Arm
{
 public:
  struct MissionControlData
  {
    enum class ArmModes : char
    {
      kTransport  = 'T',
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

  Arm(ArmJoint & rotunda, ArmJoint & shoulder, ArmJoint & elbow)
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

  void HandleConcurrentMode(MissionControlData::ArmAngles arm_angles,
                            float speed)
  {
    MoveRotunda(static_cast<float>(arm_angles.rotunda), speed);
    MoveShoulder(static_cast<float>(arm_angles.shoulder), speed);
    MoveElbow(static_cast<float>(arm_angles.elbow), speed);
  }

  void HandleMovement(MissionControlData::ArmAngles arm_angles, float speed)
  {
    switch (current_arm_mode_)
    {
      case MissionControlData::ArmModes::kConcurrent:
        HandleConcurrentMode(arm_angles, speed);
        break;
      case MissionControlData::ArmModes::kRotunda:
        MoveRotunda(static_cast<float>(arm_angles.rotunda), speed);
        break;
      case MissionControlData::ArmModes::kShoulder:
        MoveShoulder(static_cast<float>(arm_angles.shoulder), speed);
        break;
      case MissionControlData::ArmModes::kElbow:
        MoveElbow(static_cast<float>(arm_angles.elbow), speed);
        break;
      case MissionControlData::ArmModes::kTransport: TransportShoulder(); break;
      case MissionControlData::ArmModes::kHand: break;
    }
  }

  MissionControlData::ArmModes GetCurrentArmMode() const
  {
    return current_arm_mode_;
  }

  void SetCurrentArmMode(MissionControlData::ArmModes current_arm_mode)
  {
    current_arm_mode_ = current_arm_mode;
  }

  int GetRotundaPosition() const
  {
    return rotunda_.GetPosition();
  }

  int GetRotundaSpeed() const
  {
    return rotunda_.GetSpeed();
  }

  int GetShoulderPosition() const
  {
    return shoulder_.GetPosition();
  }

  int GetShoulderSpeed() const
  {
    return shoulder_.GetSpeed();
  }

  int GetElbowPosition() const
  {
    return elbow_.GetPosition();
  }

  int GetElbowSpeed() const
  {
    return elbow_.GetSpeed();
  }

  float GetRotundaOffsetAngle() const
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

  void TransportShoulder()
  {
    float transport_speed          = 10;
    float shoulder_transport_angle = 90;
    HomeArm(transport_speed);
    MoveShoulder(shoulder_transport_angle, transport_speed);
  }

  ArmJoint & rotunda_;
  ArmJoint & shoulder_;
  ArmJoint & elbow_;

  MissionControlData::ArmModes current_arm_mode_ =
      MissionControlData::ArmModes::kConcurrent;
};
}  // namespace sjsu::arm