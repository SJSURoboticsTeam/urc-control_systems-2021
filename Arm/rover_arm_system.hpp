#pragma once
#include "utility/math/units.hpp"
#include "joint.hpp"
#include "Hand/wrist_joint.hpp"
#include "../Common/rover_system.hpp"
#include <cmath>

namespace sjsu::arm
{
class RoverArmSystem : public sjsu::common::RoverSystem
{
 private:
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
  struct MissionControlData
  {
    enum class Modes : char
    {
      kDefault = 'D',
    };
    int heartbeat_count = 0;
    Modes modes         = Modes::kDefault;
    int rotunda_angle;
    int elbow_angle;
    int shoulder_angle;
    int wrist_roll;
    int wrist_pitch;
    int is_operational;
    struct Fingers
    {
      int pinky;
      int ring;
      int middle;
      int pointer;
      int thumb;
    };
    Fingers fingers;
  };
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

  void HandleArmMovement(){};

  void HomeArm()
  {
    // first getting the accelerometer readings/storing readings
    acceleration_t Rotunda_Acceleration  = Rotunda.GetAccelerometerData();
    acceleration_t Shoulder_Acceleration = Shoulder.GetAccelerometerData();
    acceleration_t Elbow_Acceleration    = Elbow.GetAccelerometerData();
    // then homing the different parts of the arm
    // HomeRotunda(); I don't think we need a home for the rotunda
    HomeShoulder(Rotunda_Acceleration, Shoulder_Acceleration);
    HomeElbow();
    HomeWrist();
  }
  /*void HomeRotunda(){

  }*/
  void HomeShoulder(acceleration_t Rotunda_Acceleration,
                    acceleration_t Shoulder_Acceleration)
  {
    float Rotunda_Angle =
        (arccos(9.81 /
                static_cast<float>(Rotunda_Acceleration.y))) *  // for angle
        (static_cast<float>(Rotunda_Acceleration.x) /
         abs(static_cast<float> Rotunda_Acceleration.x));  // for direction
    float Shoulder_Angle =
        (arccos(9.81 /
                static_cast<float>(Shoulder_Acceleration.y))) *  // for angle
        (static_cast<float>(Shoulder_Acceleration.x) /
         abs(static_cast<float> Shoulder_Acceleration.x));  // for direction
    units::angle::degree_t Offset_Angle =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Shoulder_Angle);
    Rotunda.motor.SetAngle(Offset_Angle);
  }
  void HomeElbow(acceleration_t Rotunda_Acceleration,
                 acceleration_t Elbow_Acceleration)
  {
    float Rotunda_Angle =
        (arccos(9.81 /
                static_cast<float>(Rotunda_Acceleration.y))) *  // for angle
        (static_cast<float>(Rotunda_Acceleration.x) /
         abs(static_cast<float> Rotunda_Acceleration.x));  // for direction
    float Elbow_Angle =
        (arccos(9.81 /
                static_cast<float>(Elbow_Acceleration.y))) *  // for angle
        (static_cast<float>(Elbow_Acceleration.x) /
         abs(static_cast<float>(Elbow_Acceleration.x))) *  // for direction
        (static_cast<float>(Elbow.Acceleration.y) /
         abs(static_cast<float>(Elbow_Acceleration.y)));
    units::angle::degree_t Offset_Angle =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Elbow_Angle);
    shoulder.motor.SetAngle(Offset_Angle);
  }
  void HomeWrist() {}

  void Esp(){};

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
