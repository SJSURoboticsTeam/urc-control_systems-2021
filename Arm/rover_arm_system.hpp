#pragma once
#include "utility/math/units.hpp"
#include "joint.hpp"
#include "Hand/wrist_joint.hpp"
#include "Hand/hand.hpp"
#include "../Common/rover_system.hpp"
#include <cmath>

namespace sjsu::arm
{
const char response_body_format[] =
    "\r\n\r\n{\n"
    "  \"heartbeat_count\": %d,\n"
    "  \"is_operational\": %d,\n"
    "  \"arm_speed\": %d,\n"
    "  \"rotunda_angle\": %d,\n"
    "  \"shoulder_angle\": %d,\n"
    "  \"elbow_angle\": %d,\n"
    "  \"wrist_roll\": %d,\n"
    "  \"wrist_pitch\": %d,\n"
    "  \"pinky_angle\": %d,\n"
    "  \"ring_angle\": %d,\n"
    "  \"middle_angle\": %d,\n"
    "  \"pointer_angle\": %d,\n"
    "  \"thumb_angle\": %d\n"
    "}";

class RoverArmSystem : public sjsu::common::RoverSystem
{
 public:
  struct ParseError
  {
  };
  struct MissionControlData : public RoverMissionControlData
  {
    enum class Modes : char
    {
      kHomeArm    = 'A',
      kHomeHand   = 'H',
      kConcurrent = 'C',
      kRotunda    = 'R',
      kShoulder   = 'S',
      kElbow      = 'E',
      kWrist      = 'W'

    };
    Modes modes        = Modes::kConcurrent;
    int arm_speed      = 0;
    int rotunda_angle  = 0;
    int shoulder_angle = 0;
    int elbow_angle    = 0;
    int wrist_roll     = 0;
    int wrist_pitch    = 0;

    struct Finger
    {
      int pinky_angle   = 0;
      int ring_angle    = 0;
      int middle_angle  = 0;
      int pointer_angle = 0;
      int thumb_angle   = 0;
    };
    Finger finger;
  };

  struct Acceleration
  {
    Joint::Acceleration rotunda;
    Joint::Acceleration shoulder;
    Joint::Acceleration elbow;
    WristJoint::Acceleration wrist;
  };

  RoverArmSystem(sjsu::arm::Joint & rotunda,
                 sjsu::arm::Joint & shoulder,
                 sjsu::arm::Joint & elbow,
                 sjsu::arm::WristJoint & wrist,
                 sjsu::arm::Hand & hand)
      : rotunda_(rotunda),
        shoulder_(shoulder),
        elbow_(elbow),
        wrist_(wrist),
        hand_(hand){};

  void Initialize() override
  {
    rotunda_.Initialize();
    shoulder_.Initialize();
    elbow_.Initialize();
    wrist_.Initialize();  // TODO: Move to hand class
    hand_.Initialize();
  }

  void PrintRoverData() override
  {
    printf("Arm data: \n");
    printf("Mode: %c\n", mc_data_.modes);
    printf("Arm speed: %d\n", mc_data_.arm_speed);
    printf("Rotunda Angle: %d\n", mc_data_.rotunda_angle);
    printf("Shoulder Angle: %d\n", mc_data_.shoulder_angle);
    printf("Elbow Angle: %d\n", mc_data_.elbow_angle);
    printf("Wrist Roll Angle: %d\n", mc_data_.wrist_roll);
    printf("Wrist Pitch Angle: %d\n", mc_data_.wrist_pitch);

    printf("Hand Finger Angles: \n");
    printf("Pinky Angle: %d\n", mc_data_.finger.pinky_angle);
    printf("Ring Angle: %d\n", mc_data_.finger.ring_angle);
    printf("Middle Angle: %d\n", mc_data_.finger.middle_angle);
    printf("Pointer Angle: %d\n", mc_data_.finger.pointer_angle);
    printf("Thumb Angle: %d\n", mc_data_.finger.thumb_angle);

    printf("Hand Finger Positions:\n");
    printf("Pinky Angle: %d\n", hand_.GetPinkyPosition());
    printf("Ring Angle: %d\n", hand_.GetRingPosition());
    printf("Middle Angle: %d\n", hand_.GetMiddlePosition());
    printf("Pointer Angle: %d\n", hand_.GetPointerPosition());
    printf("Thumb Angle: %d\n", hand_.GetThumbPosition());

    printf("Joints Data:\n");
    printf("Rotunda speed: %d\n", rotunda_.GetSpeed());
    printf("Rotunda position: %d\n", rotunda_.GetPosition());

    printf("Shoulder speed: %d\n", shoulder_.GetSpeed());
    printf("Shoulder position: %d\n", shoulder_.GetPosition());

    printf("Elbow speed: %d\n", elbow_.GetSpeed());
    printf("Elbow position: %d\n", elbow_.GetPosition());

    printf("Wrist pitch position: %d\n", wrist_.GetPitchPosition());
    printf("Wrist roll position: %d\n", wrist_.GetRollPosition());
  }

  std::string GETParameters() override
  {
    char request_parameter[300];
    snprintf(request_parameter, 300,
             "?heartbeat_count=%d&is_operational=%d&arm_speed=%d&battery=%d&"
             "rotunda_angle=%d&shoulder_angle=%d&elbow_angle=%d&wrist_roll=%d&"
             "wrist_pitch=%d&pinky_angle=%d&ring_angle=%d&middle_angle=%d&"
             "pointer_angle=%d&thumb_angle=%d",
             GetHeartbeatCount(), mc_data_.is_operational,
             int(mc_data_.arm_speed), state_of_charge_, rotunda_.GetPosition(),
             shoulder_.GetPosition(), elbow_.GetPosition(),
             wrist_.GetRollPosition(), wrist_.GetPitchPosition(),
             hand_.GetPinkyPosition(), hand_.GetRingPosition(),
             hand_.GetMiddlePosition(), hand_.GetPointerPosition(),
             hand_.GetThumbPosition());
    return request_parameter;
  }

  void ParseJSONResponse(std::string & response) override
  {
    int actual_arguments = sscanf(
        response.c_str(), response_body_format, &mc_data_.heartbeat_count,
        &mc_data_.is_operational, &mc_data_.arm_speed, &mc_data_.rotunda_angle,
        &mc_data_.shoulder_angle, &mc_data_.elbow_angle, &mc_data_.wrist_roll,
        &mc_data_.wrist_pitch, &mc_data_.finger.pinky_angle,
        &mc_data_.finger.ring_angle, &mc_data_.finger.middle_angle,
        &mc_data_.finger.pointer_angle, &mc_data_.finger.thumb_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  bool IsOperational()
  {
    if (mc_data_.is_operational != 1)
    {
      sjsu::LogWarning("Drive mode is not operational!");
      return false;
    }
    return true;
  }

  void MoveRotunda(float angle)
  {
    rotunda_.SetJointSpeed(mc_data_.arm_speed);
    rotunda_.SetPosition(angle);
  }

  void MoveShoulder(float angle)
  {
    shoulder_.SetJointSpeed(mc_data_.arm_speed);
    shoulder_.SetPosition(angle);
  }

  void MoveElbow(float angle)
  {
    elbow_.SetJointSpeed(mc_data_.arm_speed);
    elbow_.SetPosition(angle);
  }

  void MoveHand()
  {
    hand_.HandleHandMovement();
  }

  // TODO: implement different arm drive modes in this function with switch
  // statements
  void HandleRoverMovement() override
  {
    MoveRotunda(mc_data_.rotunda_angle);
    MoveShoulder(mc_data_.shoulder_angle);
    MoveElbow(mc_data_.elbow_angle);
    MoveHand();
  }

  void HomeArm()
  {
    UpdateAccelerations();
    HomeShoulder();
    UpdateAccelerations();
    HomeElbow();
    UpdateAccelerations();
    hand_.HomeWrist(float(rotunda_.GetOffsetAngle()));
  }

 private:
  void UpdateAccelerations()
  {
    accelerations_.rotunda = ChangeJointAccelerationIfZero(rotunda.GetAccelerometerData());
    accelerations_.shoulder = ChangeJointAccelerationIfZero(shoulder.GetAccelerometerData());
    accelerations_.elbow = ChangeJointAccelerationIfZero(elbow.GetAccelerometerData());
    accelerations_.wrist = ChangeWristJointAccelerationIfZero(wrist.GetAccelerometerData());
  }
  float CalculateShoulderHomeAngle()
  {
    float home_angle = 0;

    float acceleration_x = accelerations_.rotunda.x + accelerations_.shoulder.x;
    float acceleration_y = accelerations_.rotunda.y + accelerations_.shoulder.y;

    home_angle = float(atan(acceleration_y / acceleration_x));
    return home_angle;
  }

  float CalculateUncorrectedElbowHomeAngle()
  {
    float acceleration_x = accelerations_.rotunda.x + accelerations_.elbow.x;
    float acceleration_y = accelerations_.rotunda.y + accelerations_.elbow.y;
    float angle_without_correction =
        float(atan(acceleration_y / acceleration_x));
    return angle_without_correction;
  }

  bool InSecondQuadrantOfGraph()
  {
    if (accelerations_.elbow.x + accelerations_.rotunda.x >= 0 &&
        accelerations_.elbow.y + accelerations_.rotunda.y <= 0)
        {
          return true;
        }
    return false;
  }

  bool InThirdQuandrantOfGraph()
  if (accelerations_.elbow.x + accelerations_.rotunda.x >= 0 &&
             accelerations_.elbow.y + accelerations_.rotunda.y >= 0)
      {
        return true;
      }
  return false;

  void HomeElbow()
  {
    float home_angle;

    float angle_without_correction = CalculateUncorrectedElbowHomeAngle();
    if (InSecondQuadrantOfGraph())
    {
      home_angle = 90 - angle_without_correction;
    }
    else if (InThirdQuandrantOfGraph())
    {
      home_angle = 180 + angle_without_correction;
    }
    else
    {
      home_angle = angle_without_correction;
    }
    elbow_.SetZeroOffset(home_angle);
    MoveElbow(home_angle);
  }

  void HomeShoulder()
  {
    float home_angle = 0;

    home_angle = CalculateShoulderHomeAngle();
    shoulder_.SetZeroOffset(home_angle);
    MoveShoulder(home_angle);
  }

  int state_of_charge_                    = 90;
  MissionControlData::Modes current_mode_ = MissionControlData::Modes::kConcurrent;

  const int kExpectedArguments = 13;

 public:
  Acceleration accelerations_;
  MissionControlData mc_data_;

  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Hand hand_;
};
}  // namespace sjsu::arm