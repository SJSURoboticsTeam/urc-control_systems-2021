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

 public:
  struct MissionControlData
  {
    enum class Modes : char
    {
      kDefault = 'D',
    };
    int is_operational;
    int heartbeat_count = 0;
    Modes modes         = Modes::kDefault;
    int rotunda_angle;
    int shoulder_angle;
    int elbow_angle;
    int wrist_roll;
    int wrist_pitch;
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

  std::string GETParameters(){return "Fill";};

  std::string ParseJSONResponse(){return "Fill";};

  void HandleArmMovement(){};

  void HomeArm()
  {
    // first getting the accelerometer readings/storing readings
    Accelerometer::Acceleration_t Rotunda_Acceleration  = Rotunda.GetAccelerometerData();
    Accelerometer::Acceleration_t Shoulder_Acceleration = Shoulder.GetAccelerometerData();
    // then homing the different parts of the arm
    HomeShoulder(Rotunda_Acceleration, Shoulder_Acceleration);
    Accelerometer::Acceleration_t Elbow_Acceleration = Elbow.GetAccelerometerData();
    HomeElbow(Rotunda_Acceleration, Elbow_Acceleration);

    HomeWrist();
  }
  void HomeShoulder(Accelerometer::Acceleration_t Rotunda_Acceleration,
                    Accelerometer::Acceleration_t Shoulder_Acceleration)
  {
    float gravity = 9.81;
    float Rotunda_Angle =
        (acos(gravity /
                (static_cast<float>(Rotunda_Acceleration.y))) *  // for angle
        (static_cast<float>(Rotunda_Acceleration.x) /
         abs(static_cast<float>(Rotunda_Acceleration.x))));  // for direction
    float Shoulder_Angle =
        (acos(gravity /
                static_cast<float>(Shoulder_Acceleration.y))) *  // for angle
        (static_cast<float>(Shoulder_Acceleration.x) /
         abs(static_cast<float>(Shoulder_Acceleration.x)));  // for direction
    units::angle::degree_t Offset_Angle =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Shoulder_Angle);
    Rotunda.SetPosition(Offset_Angle);
  }
  void HomeElbow(Accelerometer::Acceleration_t Rotunda_Acceleration,
                 Accelerometer::Acceleration_t Elbow_Acceleration)
  {
    int gravity = 9.81;
    float Rotunda_Angle =
        (acos(gravity /
                static_cast<float>(Rotunda_Acceleration.y))) *  // for angle
        (static_cast<float>(Rotunda_Acceleration.x) /
         abs(static_cast<float>(Rotunda_Acceleration.x)));  // for direction
    float Elbow_Angle =
        (acos(gravity /
                static_cast<float>(Elbow_Acceleration.y))) *  // for angle
        (static_cast<float>(Elbow_Acceleration.x) /
         abs(static_cast<float>(Elbow_Acceleration.x))) *  // for direction
        (static_cast<float>(Elbow_Acceleration.y) /
         abs(static_cast<float>(Elbow_Acceleration.y)));
    units::angle::degree_t Offset_Angle =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Elbow_Angle);
    Shoulder.SetPosition(Offset_Angle);
  }
  void HomeWrist() {
    float gravity = 9.81;
  }

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
