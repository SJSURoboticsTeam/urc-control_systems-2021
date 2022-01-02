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
    int rotunda_speed;
    int rotunda_angle;
    int shoulder_speed;
    int shoulder_angle;
    int elbow_speed;
    int elbow_angle;
    int wrist_speed;
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

  std::string GETParameters()
  {
    return "Fill";
  };

  std::string ParseJSONResponse()
  {
    return "Fill";
  };

  void HandleArmMovement(){
    //rotunda
    units::angular_velocity::revolutions_per_minute_t rotunda_speed(static_cast<float>(mc_data.rotunda_speed));
    units::angle::degree_t rotunda_angle(static_cast<float>(mc_data.rotunda_angle));
    //shoulder
    units::angular_velocity::revolutions_per_minute_t shoulder_speed(static_cast<float>(mc_data.shoulder_speed));
    units::angle::degree_t shoulder_angle(static_cast<float>(mc_data.shoulder_angle));
    //elbow
    units::angular_velocity::revolutions_per_minute_t elbow_speed(static_cast<float>(mc_data.elbow_speed));
    units::angle::degree_t elbow_angle(static_cast<float>(mc_data.elbow_angle));
    //wrist
    units::angular_velocity::revolutions_per_minute_t wrist_speed(static_cast<float>(mc_data.wrist_speed));
    //units::angle::degree_t wrist_roll(static_cast<float>(mc_data.wrist_roll));
    //units::angle::degree_t wrist_pitch(static_cast<float>(mc_data.wrist_pitch));
    
    //TODO: implement different arm drive modes in this function.

    //D drive mode
    Rotunda.SetSpeed(rotunda_speed);
    Rotunda.SetPosition(rotunda_angle);
    Shoulder.SetSpeed(shoulder_speed);
    Shoulder.SetPosition(shoulder_angle);
    Elbow.SetSpeed(elbow_speed);
    Elbow.SetPosition(elbow_angle);
  }

  void HomeArm()
  {
    // first getting the accelerometer readings/storing readings
    Accelerometer::Acceleration_t Rotunda_Acceleration =
        Rotunda.GetAccelerometerData();
    Accelerometer::Acceleration_t Shoulder_Acceleration =
        Shoulder.GetAccelerometerData();
    // then homing the different parts of the arm
    HomeShoulder(Rotunda_Acceleration, Shoulder_Acceleration);

    Accelerometer::Acceleration_t Elbow_Acceleration =
        Elbow.GetAccelerometerData();
    HomeElbow(Rotunda_Acceleration, Elbow_Acceleration);

    HomeWrist();
  }
  void HomeShoulder(Accelerometer::Acceleration_t Rotunda_Acceleration,
                    Accelerometer::Acceleration_t Shoulder_Acceleration)
  {
    double gravity                = 9.81;
    double Rotunda_Acceleration_x = static_cast<double>(Rotunda_Acceleration.x);
    double Rotunda_Acceleration_y = static_cast<double>(Rotunda_Acceleration.y);
    double Shoulder_Acceleration_x =
        static_cast<double>(Shoulder_Acceleration.x);
    double Shoulder_Acceleration_y =
        static_cast<double>(Shoulder_Acceleration.y);
    if (Rotunda_Acceleration_x == 0.0)
    {  // catches division by 0
      Rotunda_Acceleration_x = .0001;
    }
    if (Rotunda_Acceleration_y == 0.0)
    {  // catches division by 0
      Rotunda_Acceleration_y = .0001;
    }
    if (Shoulder_Acceleration_x == 0.0)
    {  // catches division by 0
      Shoulder_Acceleration_x = .0001;
    }
    if (Shoulder_Acceleration_x == 0.0)
    {  // catches division by 0
      Shoulder_Acceleration_x = .0001;
    }
    double Rotunda_Angle =
        acos(gravity / Rotunda_Acceleration_y) *  // for angle
         (Rotunda_Acceleration_x /
          fabs(Rotunda_Acceleration_x));  // for direction
    double Shoulder_Angle =
        acos(gravity / Shoulder_Acceleration_y) *  // for angle
         (Shoulder_Acceleration_x /
          fabs(Shoulder_Acceleration_x));  // for direction
    units::angle::degree_t home =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Shoulder_Angle);
    Rotunda.SetPosition(home);
  }
  void HomeElbow(Accelerometer::Acceleration_t Rotunda_Acceleration,
                 Accelerometer::Acceleration_t Elbow_Acceleration)
  {
    double gravity                = 9.81;
    double Rotunda_Acceleration_x = static_cast<double>(Rotunda_Acceleration.x);
    double Rotunda_Acceleration_y = static_cast<double>(Rotunda_Acceleration.y);
    double Elbow_Acceleration_x   = static_cast<double>(Elbow_Acceleration.x);
    double Elbow_Acceleration_y   = static_cast<double>(Elbow_Acceleration.y);
    if (Rotunda_Acceleration_x == 0.0)
    {  // catches division by 0
      Rotunda_Acceleration_x = .0001;
    }
    if (Rotunda_Acceleration_y == 0.0)
    {  // catches division by 0
      Rotunda_Acceleration_y = .0001;
    }
    if (Elbow_Acceleration_x == 0.0)
    {  // catches division by 0
      Elbow_Acceleration_x = .0001;
    }
    if (Elbow_Acceleration_y == 0.0)
    {  // catches division by 0
      Elbow_Acceleration_y = .0001;
    }
    double Rotunda_Angle =
        acos(gravity / Rotunda_Acceleration_y) *  // for angle
        (Rotunda_Acceleration_x /
         fabs(Rotunda_Acceleration_x));  // for direction
    double Elbow_Angle =
        acos(gravity /
              Elbow_Acceleration_y) *  // for angle
        (Elbow_Acceleration_x /
         fabs(Elbow_Acceleration_x)) *
        (Elbow_Acceleration_y /
         fabs(Elbow_Acceleration_y));  // for direction
    units::angle::degree_t home =
        static_cast<units::angle::degree_t>(Rotunda_Angle + Elbow_Angle);
    Shoulder.SetPosition(home);
  }
  void HomeWrist(){};

  bool SyncedWithMissionControl(){};

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

  MissionControlData mc_data;
};
}  // namespace sjsu::arm
