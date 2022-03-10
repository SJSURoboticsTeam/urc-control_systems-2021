#pragma once
#include <cmath>

#include "Arm/arm.hpp"
#include "Hand/hand.hpp"
#include "Interface/hand_interface.hpp"
#include "Interface/joint_interface.hpp"
#include "../Common/Interface/rover_system_interface.hpp"

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

class RoverArmSystem : public sjsu::common::RoverSystemInterface
{
 public:
  struct ParseError
  {
  };
  struct GeneralMissionControlData : public RoverMissionControlData
  {
    int speed = 0;
  };

  RoverArmSystem(Arm & arm, Hand & hand) : arm_(arm), hand_(hand){};

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
    printf("Mode: %c\n", static_cast<char>(arm_mc_data_.arm_mode));
    printf("Arm speed: %d\n", mc_data_.speed);
    printf("Rotunda Angle: %d\n", arm_mc_data_.arm_angles.rotunda);
    printf("Shoulder Angle: %d\n", arm_mc_data_.arm_angles.shoulder);
    printf("Elbow Angle: %d\n", arm_mc_data_.arm_angles.elbow);
    printf("Wrist Roll Angle: %d\n", hand_mc_data_.wrist_data.roll);
    printf("Wrist Pitch Angle: %d\n", hand_mc_data_.wrist_data.pitch);
    printf("=========================================\n");

    printf("FINGER-DATA \n");
    printf("=========================================\n");
    printf("Hand Mode: %c\n", static_cast<char>(hand_mc_data_.hand_mode));
    printf("Pinky Angle: %d\n", hand_mc_data_.finger_angles.pinky_angle);
    printf("Ring Angle: %d\n", hand_mc_data_.finger_angles.ring_angle);
    printf("Middle Angle: %d\n", hand_mc_data_.finger_angles.middle_angle);
    printf("Pointer Angle: %d\n", hand_mc_data_.finger_angles.pointer_angle);
    printf("Thumb Angle: %d\n", hand_mc_data_.finger_angles.thumb_angle);
    printf("=========================================\n");

    hand_.PrintHandData();
    arm_.PrintArmData();
  }

  std::string CreateGETRequestParameterWithRoverStatus() override
  {
    char request_parameter[300];
    snprintf(request_parameter, 300,
             "?heartbeat_count=%d&is_operational=%d&arm_mode=%c&hand_mode=%c&"
             "arm_speed=%d&battery=%d&rotunda_angle=%d&shoulder_angle=%d&elbow_"
             "angle=%d&wrist_roll=%d&wrist_pitch=%d&pinky_angle=%d&ring_angle=%"
             "d&middle_angle=%d&pointer_angle=%d&thumb_angle=%d",
             GetHeartbeatCount(), mc_data_.is_operational,
             static_cast<char>(arm_mc_data_.arm_mode),
             static_cast<char>(hand_mc_data_.hand_mode), int(mc_data_.speed),
             state_of_charge_, arm_.GetRotundaPosition(),
             arm_.GetShoulderPosition(), arm_.GetElbowPosition(),
             hand_.GetWristRoll(), hand_.GetWristPitch(),
             hand_.GetPinkyPosition(), hand_.GetRingPosition(),
             hand_.GetMiddlePosition(), hand_.GetPointerPosition(),
             hand_.GetThumbPosition());
    return request_parameter;
  }

  void ParseMissionControlCommands(std::string & response) override
  {
    char arm_mode, hand_mode;
    int actual_arguments = sscanf(
        response.c_str(), response_body_format, &mc_data_.heartbeat_count,
        &mc_data_.is_operational, &arm_mode, &hand_mode, &mc_data_.speed,
        &arm_mc_data_.arm_angles.rotunda, &arm_mc_data_.arm_angles.shoulder,
        &arm_mc_data_.arm_angles.elbow, &hand_mc_data_.wrist_data.roll,
        &hand_mc_data_.wrist_data.pitch,
        &hand_mc_data_.finger_angles.pinky_angle,
        &hand_mc_data_.finger_angles.ring_angle,
        &hand_mc_data_.finger_angles.middle_angle,
        &hand_mc_data_.finger_angles.pointer_angle,
        &hand_mc_data_.finger_angles.thumb_angle);

    arm_mc_data_.arm_mode   = Arm::MissionControlData::ArmModes{ arm_mode };
    hand_mc_data_.hand_mode = Hand::MissionControlData::HandModes{ hand_mode };
    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  void HomeArmSystem()
  {
    arm_.HomeArm(static_cast<float>(mc_data_.speed));
    hand_.HomeHand(arm_.GetRotundaOffsetAngle(),
                   static_cast<float>(mc_data_.speed));
  }

  // TODO: implement different arm drive modes in this function with switch
  // statements
  void HandleRoverCommands() override
  {
    if (arm_mc_data_.arm_mode != arm_.GetCurrentArmMode())
    {
      arm_.SetCurrentArmMode(arm_mc_data_.arm_mode);
    }
    if (hand_mc_data_.hand_mode != hand_.GetCurrentHandMode())
    {
      hand_.SetCurrentHandMode(hand_mc_data_.hand_mode);
    }

    arm_.HandleMovement(arm_mc_data_.arm_angles,
                        static_cast<float>(mc_data_.speed));
    hand_.HandleMovement(hand_mc_data_, static_cast<float>(mc_data_.speed));
  }

  GeneralMissionControlData GetMCData() const
  {
    return mc_data_;
  }

  Arm::MissionControlData GetArmMCData() const
  {
    return arm_mc_data_;
  }

  Hand::MissionControlData GetHandMCData() const
  {
    return hand_mc_data_;
  }

 private:
  int state_of_charge_         = 90;
  const int kExpectedArguments = 15;

  GeneralMissionControlData mc_data_;
  Arm::MissionControlData arm_mc_data_;
  Hand::MissionControlData hand_mc_data_;

  Arm arm_;
  Hand hand_;
};
}  // namespace sjsu::arm