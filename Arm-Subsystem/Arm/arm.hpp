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
    printf("%-10s%-10d%-10d\n", "ROTUNDA", rotunda_.GetPosition(),
           rotunda_.GetSpeed());
    printf("%-10s%-10d%-10d\n", "SHOULDER", shoulder_.GetPosition(),
           shoulder_.GetSpeed());
    printf("%-10s%-10d%-10d\n", "ELBOW", elbow_.GetPosition(),
           elbow_.GetSpeed());
  }

  void HomeArm(float speed)
  {
    HomeShoulder(speed);
    HomeElbow(speed);
  }

  void HomeShoulder(float speed)
  {
    float home_angle = CalculateShoulderHomeAngle();

    shoulder_.SetOffset(home_angle);
    MoveShoulder(home_angle, speed);
  }

  void HomeElbow(float speed)
  {
    float home_angle = CalculateElbowHomeAngle();

    elbow_.SetOffset(home_angle);
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
      case MissionControlData::ArmModes::kTransport: HandleTransportMode(); break;
      case MissionControlData::ArmModes::kHand: break;
    }
  }

  MissionControlData::ArmModes GetCurrentArmMode() const
  {
    return current_arm_mode_;
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

  void StopArm()
  {
    rotunda_.SetJointSpeed(0);
    shoulder_.SetJointSpeed(0);
    elbow_.SetJointSpeed(0);
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
    UpdateAccelerations();
    float acceleration_x = rotunda_.acceleration_.x + shoulder_.acceleration_.x;
    float acceleration_y = rotunda_.acceleration_.y + shoulder_.acceleration_.y;

    float shoulder_home = float(atan(acceleration_x / acceleration_y));
    return shoulder_home;
  }

  float CalculateElbowHomeAngle()
  {
    UpdateAccelerations();
    float acceleration_x = rotunda_.acceleration_.x + elbow_.acceleration_.x;
    float acceleration_y = rotunda_.acceleration_.y + elbow_.acceleration_.y;
    float elbow_home     = float(atan(acceleration_y / acceleration_x));
    return elbow_home;
  }

  void UpdateAccelerations()
  {
    rotunda_.GetAccelerometerData();
    shoulder_.GetAccelerometerData();
    elbow_.GetAccelerometerData();
  }

  void HandleTransportMode()
  {
    float transport_speed          = 10;
    float elbow_transport_angle    = CalculateElbowHomeAngle();
    float shoulder_transport_angle = 90;

    MoveElbow(elbow_transport_angle, transport_speed);
    MoveShoulder(shoulder_transport_angle, transport_speed);
  }

  ArmJoint & rotunda_;
  ArmJoint & shoulder_;
  ArmJoint & elbow_;

  MissionControlData::ArmModes current_arm_mode_ =
      MissionControlData::ArmModes::kConcurrent;
};
}  // namespace sjsu::arm