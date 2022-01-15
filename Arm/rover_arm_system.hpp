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
      kDefault = 'D',
    };
    Modes modes        = Modes::kDefault;
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
    return;
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

  void SetLerpSpeed(float target_speed){
    std:lerp(mc_data_.arm_speed, target_speed, .5)
  }

  void MoveRotunda(double angle)
    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  /// Checks that the rover is operational
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

  void HandleRoverMovement() override
  {
    // TODO: implement different arm drive modes in this function
    MoveRotunda(mc_data_.rotunda_angle);
    MoveShoulder(mc_data_.shoulder_angle);
    MoveElbow(mc_data_.elbow_angle);
  }

  void MoveWrist(){};

  void HomeArm()
  {
    UpdateRotundaAcceleration();
    UpdateShoulderAcceleration();
    HomeShoulder();
    UpdateElbowAcceleration();
    HomeElbow();
    hand_.HomeWrist(float(rotunda_.GetOffsetAngle()));
  }

  void HomeShoulder()
  {
    float home_angle = 0;
    // TODO: std might have solution for this e.g. std::clamp or std::max?
    ChangeIfZero(accelerations_.rotunda.x);
    ChangeIfZero(accelerations_.rotunda.y);
    ChangeIfZero(accelerations_.shoulder.x);
    ChangeIfZero(accelerations_.shoulder.y);

    // TODO:  Verify we don't need compliment value of shoulder
    float acceleration_x = accelerations_.rotunda.x + accelerations_.shoulder.x;
    float acceleration_y = accelerations_.rotunda.y + accelerations_.shoulder.y;

    home_angle = float(atan(acceleration_y / acceleration_x));
    shoulder_.SetZeroOffset(home_angle);
    MoveShoulder(home_angle);
  }

  void HomeElbow()
  {
    float home_angle = 0;
    // TODO: std might have solution for this e.g. std::clamp or std::max?
    ChangeIfZero(accelerations_.rotunda.x);
    ChangeIfZero(accelerations_.rotunda.y);
    ChangeIfZero(accelerations_.elbow.x);
    ChangeIfZero(accelerations_.elbow.y);

    // TODO:  Verify we don't need compliment value of shoulder
    float acceleration_x = accelerations_.rotunda.x + accelerations_.elbow.x;
    float acceleration_y = accelerations_.rotunda.y + accelerations_.elbow.y;
    float angle_without_correction =
        float(atan(acceleration_y / acceleration_x));

    // TODO: Bool helper functions for checking which quadrant correction to use
    if (accelerations_.elbow.x + accelerations_.rotunda.x >= 0 &&
        accelerations_.elbow.y + accelerations_.rotunda.y <= 0)
    {
      // if the elbow is in the second quadrant of a graph, add 90 to the angle
      home_angle = 90 - angle_without_correction;
    }
    // if the elbow is in the third quadrant of a graph, add 180 to the angle
    else if (accelerations_.elbow.x + accelerations_.rotunda.x >= 0 &&
             accelerations_.elbow.y + accelerations_.rotunda.y >= 0)
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

  // TODO: need to work on valid movement durring in person work shop
  bool CheckValidMovement()
  {
    return true;
  }

  void Calibrate()
  {
    return;
  }

  void UpdateRotundaAcceleration()
  {
    accelerations_.rotunda = rotunda_.GetAccelerometerData();
  }

  void UpdateShoulderAcceleration()
  {
    accelerations_.shoulder = shoulder_.GetAccelerometerData();
  }

  void UpdateElbowAcceleration()
  {
    accelerations_.elbow = elbow_.GetAccelerometerData();
  }

 private:
  /// Checks if value is zero. If it's zero make it not zero
  void ChangeIfZero(float & acceleration)
  {
    if (acceleration == 0)
    {
      acceleration = 0.0001;
    }
  }

  /// TODO: test if we even need this function
  float FindComplimentValue()
  {
    return 0;
  }

  int state_of_charge_                    = 90;
  MissionControlData::Modes current_mode_ = MissionControlData::Modes::kDefault;

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