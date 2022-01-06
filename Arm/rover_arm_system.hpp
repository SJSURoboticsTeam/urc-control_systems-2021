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
 public:
  struct MissionControlData : public RoverMissionControlData
  {
    enum class Modes : char
    {
      kDefault = 'D',
    };
    Modes modes = Modes::kDefault;
    double rotunda_speed;
    double rotunda_angle;
    double shoulder_speed;
    double shoulder_angle;
    double elbow_speed;
    double elbow_angle;
    double wrist_speed;
    double wrist_roll;
    double wrist_pitch;
    struct Finger
    {
      double pinky;
      double ring;
      double middle;
      double pointer;
      double thumb;
    };
    Finger finger;
  };

  RoverArmSystem(sjsu::arm::Joint & rotunda,
                 sjsu::arm::Joint & shoulder,
                 sjsu::arm::Joint & elbow,
                 sjsu::arm::WristJoint wrist)
      : rotunda_(rotunda), shoulder_(shoulder), elbow_(elbow), wrist_(wrist)
  {
  }
  void Initialize()
  {
    rotunda_.Initialize();
    shoulder_.Initialize();
    elbow_.Initialize();
    wrist_.Initialize();
  }

  void PrintRoverData(){

  };

  std::string GETParameters()
  {
    return "Fill";
  };

  std::string ParseJSONResponse()
  {
    return "Fill";
  };

  bool SyncedWithMissionControl(){
    bool fill;
    return fill;
  };

  void MoveRotunda(double speed, double angle){
    try
    {
      if(!CheckValidMovement())
      {
        throw(angle);
      }
      rotunda_.SetSpeed(speed);
      rotunda_.SetPosition(angle);
      }
      catch(double large_angle)
      {
        sjsu::LogError("Error Moving Rotunda, Requested Angle Is Too Large");
      }
  }

  void MoveShoulder(double speed, double angle){
    try
    {
      if(!CheckValidMovement())
      {
        throw(angle);
      }
      shoulder_.SetSpeed(speed);
      shoulder_.SetPosition(angle);
      }
      catch(double large_angle)
      {
        sjsu::LogError("Error Moving Shoulder, Requested Angle Is Too Large");
      }
  }

  void MoveElbow(double speed, double angle){
    try
    {
      if(!CheckValidMovement())
      {
        throw(angle);
      }
      elbow_.SetSpeed(speed);
      elbow_.SetPosition(angle);
      }
      catch(units::angle::degree_t large_angle)
      {
        sjsu::LogError("Error Moving Elbow, Requested Angle Is Too Large");
      }
  }

  void HandleArmMovement()
  {
    // TODO: implement different arm drive modes in this function.

    // D drive mode
    MoveRotunda(mc_data_.rotunda_speed, mc_data_.rotunda_angle);
    MoveShoulder(mc_data_.shoulder_speed, mc_data_.shoulder_angle);
    MoveElbow(mc_data_.elbow_speed, mc_data_.elbow_angle);
  }

  void MoveWrist()
  {

  }

  void HomeAll()
  {
    // first reading the accelerometer data
    Joint::Acceleration rotunda_acceleration =
        rotunda_.GetAccelerometerData();
    Joint::Acceleration shoulder_acceleration =
        shoulder_.GetAccelerometerData();
    HomeShoulder(rotunda_acceleration, shoulder_acceleration);

    Joint::Acceleration elbow_acceleration =
        elbow_.GetAccelerometerData();
    HomeElbow(rotunda_acceleration, elbow_acceleration);

    HomeWrist();
  }

  void HomeShoulder(Joint::Acceleration rotunda_acceleration,
                    Joint::Acceleration shoulder_acceleration)
  {
    // Make this into a helper function, this checks to see if any of the acceleration values are 0, and if they are it will change
    // to something close to 0
    double gravity = 9.81;
    if (rotunda_acceleration.x == 0.0)
    {  // catches division by 0
      rotunda_acceleration.x = .0001;
    }
    if (rotunda_acceleration.y == 0.0)
    {  // catches division by 0
      rotunda_acceleration.y = .0001;
    }
    if (shoulder_acceleration.x == 0.0)
    {  // catches division by 0
      shoulder_acceleration.x = .0001;
    }
    if (shoulder_acceleration.y == 0.0)
    {  // catches division by 0
      shoulder_acceleration.y = .0001;
    }
//TODO: clean up calculations into helper functions to make it more legible
    double rotunda_angle =
        acos(gravity / rotunda_acceleration.y) *  // for angle
        (rotunda_acceleration.x /
         fabs(rotunda_acceleration.x));  // for direction
    double shoulder_angle =
        acos(gravity / shoulder_acceleration.y) *  // for angle
        (shoulder_acceleration.x /
         fabs(shoulder_acceleration.x));  // for direction
    double home = rotunda_angle + shoulder_angle;
    MoveShoulder(10.0, home);
  }

  void HomeElbow(Joint::Acceleration rotunda_acceleration,
                 Joint::Acceleration elbow_acceleration)
  {
    double gravity                = 9.81;
    if (rotunda_acceleration.x == 0.0)
    {  // catches division by 0
      rotunda_acceleration.x = .0001;
    }
    if (rotunda_acceleration.y == 0.0)
    {  // catches division by 0
      rotunda_acceleration.y = .0001;
    }
    if (elbow_acceleration.x == 0.0)
    {  // catches division by 0
      elbow_acceleration.x = .0001;
    }
    if (elbow_acceleration.y == 0.0)
    {  // catches division by 0
      elbow_acceleration.y = .0001;
    }
    double rotunda_angle =
        acos(gravity / rotunda_acceleration.y) *  // for angle
        (rotunda_acceleration.x /
         fabs(rotunda_acceleration.x));  // for direction
    double elbow_angle =
        acos(gravity / elbow_acceleration.y) *  // for angle
        (elbow_acceleration.x / fabs(elbow_acceleration.x)) *
        (elbow_acceleration.y / fabs(elbow_acceleration.y));  // for direction
    double home = rotunda_angle + elbow_angle;
    MoveShoulder(10.0, home);
  }

  void HomeWrist(){};

  void Esp(){};

//TODO: need to work on valid movement
  bool CheckValidMovement(){
    bool fill;
    return fill;
  }

  void Calibrate(){};

  void SendUpdateToMC(){};

  void PrintArmMode(){};

 private:
  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
  sjsu::arm::WristJoint & wrist_;
  MissionControlData mc_data_;
};
}  // namespace sjsu::arm
