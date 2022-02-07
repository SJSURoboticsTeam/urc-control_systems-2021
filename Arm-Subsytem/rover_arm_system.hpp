#pragma once

#include "utility/math/units.hpp"
#include "joint.hpp"
#include "arm_joint.hpp"
#include "Hand/hand.hpp"
#include "../Common/heartbeat.hpp"
#include "../Common/rover_system.hpp"
#include "Arm/human_arm.hpp"

#include <cmath>

namespace sjsu::arm
{
const char response_body_format[] =
    "\r\n\r\n{\n"
    "  \"heartbeat_count\": %d,\n"
    "  \"is_operational\": %d,\n"
    "  \"arm_mode\": \"%c\",\n"
    "  \"hand_mode\": \"%c\",\n"
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
    //handdata will be moved to the hand class soon
    struct HandData
    {
      enum class HandModes : char
      {
        kPitch      = 'P',
        kRoll       = 'R',
        kClose      = 'F',
        kOpen       = 'O',
        kConcurrent = 'C'
      };
      HandModes hand_mode = HandModes::kConcurrent;

      struct Finger
      {
        int pinky_angle   = 0;
        int ring_angle    = 0;
        int middle_angle  = 0;
        int pointer_angle = 0;
        int thumb_angle   = 0;
      };
      Finger finger;

      int wrist_roll  = 0;
      int wrist_pitch = 0;
    };
    HandData hand;
    int arm_speed = 0;
  };

  RoverArmSystem(HumanArm & arm, Hand & hand) : arm_(arm), hand_(hand){};

  void Initialize() override
  {
    arm_.Initialize();
    hand_.Initialize();
  }

  void PrintRoverData() override
  {
    printf("SERVER-DATA");
    printf("=========================================\n");
    printf("Operational: %d\n", mc_data_.heartbeat_count);
    printf("Operational: %d\n", mc_data_.is_operational);
    printf("=========================================\n");

    printf("ARM-DATA\n");
    printf("=========================================\n");
    printf("Mode: %c\n", arm_mc_data_.arm_mode);
    printf("Arm speed: %d\n", mc_data_.arm_speed);
    printf("Rotunda Angle: %d\n", arm_mc_data_.arm_angles.rotunda);
    printf("Shoulder Angle: %d\n", arm_mc_data_.arm_angles.shoulder);
    printf("Elbow Angle: %d\n", arm_mc_data_.arm_angles.elbow);
    printf("Wrist Roll Angle: %d\n", mc_data_.hand.wrist_roll);
    printf("Wrist Pitch Angle: %d\n", mc_data_.hand.wrist_pitch);
    printf("=========================================\n");

    printf("HAND-FINGER-ANGLES \n");
    printf("=========================================\n");
    printf("Hand Mode: %c\n", mc_data_.hand.hand_mode);
    printf("Pinky Angle: %d\n", mc_data_.hand.finger.pinky_angle);
    printf("Ring Angle: %d\n", mc_data_.hand.finger.ring_angle);
    printf("Middle Angle: %d\n", mc_data_.hand.finger.middle_angle);
    printf("Pointer Angle: %d\n", mc_data_.hand.finger.pointer_angle);
    printf("Thumb Angle: %d\n", mc_data_.hand.finger.thumb_angle);
    printf("=========================================\n");

    hand_.PrintHandData();
    arm_.PrintArmData();
  }

  std::string GETParameters() override
  {
    char request_parameter[300];
    snprintf(request_parameter, 300,
             "?heartbeat_count=%d&is_operational=%d&arm_mode=%c&hand_mode=%c&"
             "arm_speed=%d&battery=%d&"
             "rotunda_angle=%d&shoulder_angle=%d&elbow_angle=%d&wrist_roll=%d&"
             "wrist_pitch=%d&pinky_angle=%d&ring_angle=%d&middle_angle=%d&"
             "pointer_angle=%d&thumb_angle=%d",
             GetHeartbeatCount(), mc_data_.is_operational,
             char(arm_mc_data_.arm_mode), char(mc_data_.hand.hand_mode),
             int(mc_data_.arm_speed), state_of_charge_,
             arm_.GetRotundaPosition(), arm_.GetShoulderPosition(),
             arm_.GetElbowPosition(), hand_.GetWristRoll(),
             hand_.GetWristPitch(), hand_.GetPinkyPosition(),
             hand_.GetRingPosition(), hand_.GetMiddlePosition(),
             hand_.GetPointerPosition(), hand_.GetThumbPosition());
    return request_parameter;
  }

  void ParseJSONResponse(std::string & response) override
  {
    int actual_arguments = sscanf(
        response.c_str(), response_body_format, &mc_data_.heartbeat_count,
        &mc_data_.is_operational, &arm_mc_data_.arm_mode, &mc_data_.hand.hand_mode,
        &mc_data_.arm_speed, &arm_mc_data_.arm_angles.rotunda, &arm_mc_data_.arm_angles.shoulder,
        &arm_mc_data_.arm_angles.elbow, &mc_data_.hand.wrist_roll, &mc_data_.hand.wrist_pitch,
        &mc_data_.hand.finger.pinky_angle, &mc_data_.hand.finger.ring_angle,
        &mc_data_.hand.finger.middle_angle, &mc_data_.hand.finger.pointer_angle,
        &mc_data_.hand.finger.thumb_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  //move to rover_system soon
  bool IsOperational()
  {
    if (mc_data_.is_operational != 1)
    {
      sjsu::LogWarning("Drive mode is not operational!");
      return false;
    }
    return true;
  }

  //is this needed?
  void MoveHand(float thumb,
                float pointer,
                float middle,
                float ring,
                float pinky,
                float pitch,
                float roll)
  {
    hand_.HandleHandMovement(mc_data_.arm_speed, thumb, pointer, middle, ring,
                             pinky, pitch, roll);
  }

  void HomeArmSystem()
  {
    arm_.HomeArm(mc_data_.arm_speed);
    hand_.HomeHand(mc_data_.arm_speed, arm_.GetRotundaOffsetAngle());
  }

  // TODO: implement different arm drive modes in this function with switch
  // statements
  void HandleRoverMovement() override
  {
    if (arm_mc_data_.arm_mode != arm_.GetCurrentArmMode())
    {
      arm_.SetCurrentArmMode(arm_mc_data_.arm_mode);
    }
    if (mc_data_.hand.hand_mode != current_hand_mode_)
    {
      current_hand_mode_ = mc_data_.hand.hand_mode;
    }
    arm_.HandleMovement(arm_mc_data_.arm_angles.rotunda, arm_mc_data_.arm_angles.shoulder, arm_mc_data_.arm_angles.elbow, mc_data_.arm_speed);
    HandleHandModes();
  }

 private:
  void HandleHandConcurrentMode()
  {
    MoveHand(mc_data_.hand.finger.thumb_angle,
             mc_data_.hand.finger.pointer_angle,
             mc_data_.hand.finger.middle_angle, mc_data_.hand.finger.ring_angle,
             mc_data_.hand.finger.pinky_angle, mc_data_.hand.wrist_roll,
             mc_data_.hand.wrist_pitch);
  }

  void HandleHandModes()
  {
    switch (current_hand_mode_)
    {
      case MissionControlData::HandData::HandModes::kPitch:
        hand_.SetWristPitchPosition(mc_data_.arm_speed,
                                    mc_data_.hand.wrist_pitch);
        break;
      case MissionControlData::HandData::HandModes::kRoll:
        hand_.SetWristRollPosition(mc_data_.arm_speed,
                                   mc_data_.hand.wrist_roll);
        break;
      case MissionControlData::HandData::HandModes::kClose:
        hand_.CloseHand(mc_data_.arm_speed);
        break;
      case MissionControlData::HandData::HandModes::kOpen:
        hand_.OpenHand(mc_data_.arm_speed);
        break;
      case MissionControlData::HandData::HandModes::kConcurrent:
        MoveHand(mc_data_.hand.finger.thumb_angle,
                 mc_data_.hand.finger.pointer_angle,
                 mc_data_.hand.finger.middle_angle,
                 mc_data_.hand.finger.ring_angle,
                 mc_data_.hand.finger.pinky_angle, mc_data_.hand.wrist_roll,
                 mc_data_.hand.wrist_pitch);
        break;
    }
  }

  int state_of_charge_         = 90;
  const int kExpectedArguments = 15;

 public:
  HumanArm::MissionControlArmData arm_mc_data_;
  MissionControlData mc_data_;

  MissionControlData::HandData::HandModes current_hand_mode_= MissionControlData::HandData::HandModes::kConcurrent;

  Hand hand_;
  HumanArm arm_;
};
}  // namespace sjsu::arm