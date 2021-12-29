#pragma once
#include "utility/math/units.hpp"
#include "joint.hpp"
#include "Hand/wrist_joint.hpp"
#include "../Common/rover_system.hpp"

namespace sjsu::arm
{
class RoverArmSystem : public sjsu::common::RoverSystem
{
 private:
  // isOperational defines if the arm should be allowed to move or if it should
  // be in its resting 'off' position.
  bool isOperational;

  sjsu::arm::Joint & Rotunda;
  sjsu::arm::Joint & Shoulder;
  sjsu::arm::Joint & Elbow;
  sjsu::arm::WristJoint & Wrist;

  // The target angle for each of the joints
  units::angle::degree_t shoulder_pos;
  units::angle::degree_t elbow_pos;
  units::angle::degree_t rotunda_pos;
  units::angle::degree_t wrist_roll_pos;
  units::angle::degree_t wrist_pitch_pos;

 public:
  RoverArmSystem(sjsu::arm::Joint & rotunda,
                 sjsu::arm::Joint & shoulder,
                 sjsu::arm::Joint & elbow,
                 sjsu::arm::WristJoint wrist)
      : Rotunda(rotunda), Shoulder(shoulder), Elbow(elbow), Wrist(wrist)
  {
  }
  void Initialize()
  {
    Rotunda.Initialize();
    Shoulder.Initialize();
    Elbow.Initialize();
    Wrist.Initialize();
  }

  void PrintRoverData(){};

  std::string GETParameters(){};

  std::string ParseJSONResponse(){};

  void HandleRoverMovement(){};

  void Homing(){};

  void Esp(){};

  void ArmMovement(){};

  void ExtendArm(){};

  void RotateArm(){};

  void CheckValidMovement(){};

  void GraspHand(){};

  void Calibrate(){};

  void SendUpdateToMC(){};

  void PrintArmMode(){};

  void MoveRotand(){};

  void MoveShoulder(){};

  void MoveElbow(){};

  void MoveWrist(){};
};
}  // namespace sjsu::arm
