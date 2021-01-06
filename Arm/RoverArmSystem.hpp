#include "utility/units.hpp"
#include "joint.hpp"
#include "wrist_joint.hpp"

namespace sjsu
{
class RoverArmSystem
{
 private:
  // isOperational defines if the arm should be allowed to move or if it should
  // be in its resting 'off' position.
  bool isOperational;

  sjsu::arm::joint Rotunda;
  sjsu::arm::joint Shoulder;
  sjsu::arm::joint Elbow;
  sjsu::arm::wrist_joint Wrist;

  // The target angle for each of the joints
  units::angle::degree_t shoulder_pos;
  units::angle::degree_t elbow_pos;
  units::angle::degree_t rotunda_pos;
  units::angle::degree_t wrist_roll_pos;
  units::angle::degree_t wrist_pitch_pos;

 public:
  RoverArmSystem();
  bool Home();
  bool Initialize();
  bool Get_data();
  bool Move_arm();
};

RoverArmSystem::RoverArmSystem()
{
  Rotunda  = sjsu::arm::joint();
  Shoulder = sjsu::arm::joint();
  Elbow    = sjsu::arm::joint();
  Wrist    = sjsu::arm::wrist_joint();
}

bool RoverArmSystem::Home()
{
  return true;
}
bool RoverArmSystem::Initialize()
{
  return true;
}
bool RoverArmSystem::Get_data()
{
  return true;
}
bool RoverArmSystem::Move_arm()
{
  return true;
}
}  // namespace sjsu
