#pragma once

#include "utility/math/units.hpp"
#include "joint.hpp"
#include "Hand/human_hand.hpp"
#include "../Common/heartbeat.hpp"
#include "../Common/rover_system.hpp"
#include "Arm/human_arm.hpp"
#include "Interface/hand.hpp"

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
  struct GeneralMissionControlData : public RoverMissionControlData
  {
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
    printf("Wrist Roll Angle: %d\n", hand_mc_data_.wrist_data.roll);
    printf("Wrist Pitch Angle: %d\n", hand_mc_data_.wrist_data.pitch);
    printf("=========================================\n");

    printf("FINGER-DATA \n");
    printf("=========================================\n");
    printf("Hand Mode: %c\n", hand_mc_data_.hand_mode);
    printf("Pinky Angle: %d\n", hand_mc_data_.fingers.pinky_angle);
    printf("Ring Angle: %d\n", hand_mc_data_.fingers.ring_angle);
    printf("Middle Angle: %d\n", hand_mc_data_.fingers.middle_angle);
    printf("Pointer Angle: %d\n", hand_mc_data_.fingers.pointer_angle);
    printf("Thumb Angle: %d\n", hand_mc_data_.fingers.thumb_angle);
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
             char(arm_mc_data_.arm_mode), char(hand_mc_data_.hand_mode),
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
        &mc_data_.is_operational, &arm_mc_data_.arm_mode,
        &hand_mc_data_.hand_mode, &mc_data_.arm_speed,
        &arm_mc_data_.arm_angles.rotunda, &arm_mc_data_.arm_angles.shoulder,
        &arm_mc_data_.arm_angles.elbow, &hand_mc_data_.wrist_data.roll,
        &hand_mc_data_.wrist_data.pitch, &hand_mc_data_.fingers.pinky_angle,
        &hand_mc_data_.fingers.ring_angle, &hand_mc_data_.fingers.middle_angle,
        &hand_mc_data_.fingers.pointer_angle,
        &hand_mc_data_.fingers.thumb_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  void HomeArmSystem()
  {
    arm_.HomeArm(mc_data_.arm_speed);
    hand_.HomeHand(arm_.GetRotundaOffsetAngle(), mc_data_.arm_speed);
  }

  // TODO: implement different arm drive modes in this function with switch
  // statements
  void HandleRoverMovement() override
  {
    if (arm_mc_data_.arm_mode != arm_.GetCurrentArmMode())
    {
      arm_.SetCurrentArmMode(arm_mc_data_.arm_mode);
    }
    if (hand_mc_data_.hand_mode != hand_.GetCurrentHandMode())
    {
      hand_.SetCurrentHandMode(hand_mc_data_.hand_mode);
    }

    arm_.HandleMovement(arm_mc_data_.arm_angles, mc_data_.arm_speed);
    hand_.HandleMovement(hand_mc_data_, mc_data_.arm_speed);
  }

 private:
  int state_of_charge_         = 90;
  const int kExpectedArguments = 15;

  GeneralMissionControlData mc_data_;
  HumanArm::MissionControlData arm_mc_data_;
  Hand::MissionControlData hand_mc_data_;

  HumanArm arm_;
  Hand hand_;
};
}  // namespace sjsu::arm