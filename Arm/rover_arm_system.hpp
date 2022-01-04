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
    int rotunda_speed;
    int rotunda_angle;
    int shoulder_speed;
    int shoulder_angle;
    int elbow_speed;
    int elbow_angle;
    int wrist_speed;
    int wrist_roll;
    int wrist_pitch;
    struct Finger
    {
      int pinky;
      int ring;
      int middle;
      int pointer;
      int thumb;
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

  void MoveRotunda(units::angular_velocity::revolutions_per_minute_t speed, units::angle::degree_t angle){
    try
    {
      if(!CheckValidMovement())
      {
        throw(angle);
      }
      rotunda_.SetSpeed(speed);
      rotunda_.SetPosition(angle);
      }
      catch(units::angle::degree_t large_angle)
      {
        sjsu::LogError("Error Moving Rotunda, Requested Angle Is Too Large");
      }
  }

  void MoveShoulder(units::angular_velocity::revolutions_per_minute_t speed, units::angle::degree_t angle){
    try
    {
      if(!CheckValidMovement())
      {
        throw(angle);
      }
      shoulder_.SetSpeed(speed);
      shoulder_.SetPosition(angle);
      }
      catch(units::angle::degree_t large_angle)
      {
        sjsu::LogError("Error Moving Shoulder, Requested Angle Is Too Large");
      }
  }

  void MoveElbow(units::angular_velocity::revolutions_per_minute_t speed, units::angle::degree_t angle){
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
    // rotunda
    units::angular_velocity::revolutions_per_minute_t rotunda_speed(
        static_cast<float>(mc_data.rotunda_speed));
    units::angle::degree_t rotunda_angle(
        static_cast<float>(mc_data.rotunda_angle));
    // shoulder
    units::angular_velocity::revolutions_per_minute_t shoulder_speed(
        static_cast<float>(mc_data.shoulder_speed));
    units::angle::degree_t shoulder_angle(
        static_cast<float>(mc_data.shoulder_angle));
    // elbow
    units::angular_velocity::revolutions_per_minute_t elbow_speed(
        static_cast<float>(mc_data.elbow_speed));
    units::angle::degree_t elbow_angle(static_cast<float>(mc_data.elbow_angle));
    // wrist
    units::angular_velocity::revolutions_per_minute_t wrist_speed(
        static_cast<float>(mc_data.wrist_speed));
    // units::angle::degree_t
    // wrist_roll(static_cast<float>(mc_data.wrist_roll));
    // units::angle::degree_t
    // wrist_pitch(static_cast<float>(mc_data.wrist_pitch));

    // TODO: implement different arm drive modes in this function.

    // D drive mode
    MoveRotunda(rotunda_speed, rotunda_angle);
    MoveShoulder(shoulder_speed, shoulder_angle);
    MoveElbow(elbow_speed, elbow_angle);
  }

  void MoveWrist(){};

  void HomeAll()
  {
    // first getting the accelerometer readings/storing readings
    Accelerometer::Acceleration_t rotunda_acceleration =
        rotunda_.GetAccelerometerData();
    Accelerometer::Acceleration_t shoulder_acceleration =
        shoulder_.GetAccelerometerData();
    // then homing the different parts of the arm
    HomeShoulder(rotunda_acceleration, shoulder_acceleration);

    Accelerometer::Acceleration_t elbow_acceleration =
        elbow_.GetAccelerometerData();
    HomeElbow(rotunda_acceleration, elbow_acceleration);

    HomeWrist();
  }

  void HomeShoulder(Accelerometer::Acceleration_t rotunda_acceleration,
                    Accelerometer::Acceleration_t shoulder_acceleration)
  {
    double gravity                = 9.81;
    double rotunda_acceleration_x = static_cast<double>(rotunda_acceleration.x);
    double rotunda_acceleration_y = static_cast<double>(rotunda_acceleration.y);
    double shoulder_acceleration_x =
        static_cast<double>(shoulder_acceleration.x);
    double shoulder_acceleration_y =
        static_cast<double>(shoulder_acceleration.y);
    if (rotunda_acceleration_x == 0.0)
    {  // catches division by 0
      rotunda_acceleration_x = .0001;
    }
    if (rotunda_acceleration_y == 0.0)
    {  // catches division by 0
      rotunda_acceleration_y = .0001;
    }
    if (shoulder_acceleration_x == 0.0)
    {  // catches division by 0
      shoulder_acceleration_x = .0001;
    }
    if (shoulder_acceleration_y == 0.0)
    {  // catches division by 0
      shoulder_acceleration_y = .0001;
    }
    double rotunda_angle =
        acos(gravity / rotunda_acceleration_y) *  // for angle
        (rotunda_acceleration_x /
         fabs(rotunda_acceleration_x));  // for direction
    double shoulder_angle =
        acos(gravity / shoulder_acceleration_y) *  // for angle
        (shoulder_acceleration_x /
         fabs(shoulder_acceleration_x));  // for direction
    units::angle::degree_t home =
        static_cast<units::angle::degree_t>(rotunda_angle + shoulder_angle);
    rotunda_.SetPosition(home);
  }

  void HomeElbow(Accelerometer::Acceleration_t rotunda_acceleration,
                 Accelerometer::Acceleration_t elbow_acceleration)
  {
    double gravity                = 9.81;
    double rotunda_acceleration_x = static_cast<double>(rotunda_acceleration.x);
    double rotunda_acceleration_y = static_cast<double>(rotunda_acceleration.y);
    double elbow_acceleration_x   = static_cast<double>(elbow_acceleration.x);
    double elbow_acceleration_y   = static_cast<double>(elbow_acceleration.y);
    if (rotunda_acceleration_x == 0.0)
    {  // catches division by 0
      rotunda_acceleration_x = .0001;
    }
    if (rotunda_acceleration_y == 0.0)
    {  // catches division by 0
      rotunda_acceleration_y = .0001;
    }
    if (elbow_acceleration_x == 0.0)
    {  // catches division by 0
      elbow_acceleration_x = .0001;
    }
    if (elbow_acceleration_y == 0.0)
    {  // catches division by 0
      elbow_acceleration_y = .0001;
    }
    double rotunda_angle =
        acos(gravity / rotunda_acceleration_y) *  // for angle
        (rotunda_acceleration_x /
         fabs(rotunda_acceleration_x));  // for direction
    double elbow_angle =
        acos(gravity / elbow_acceleration_y) *  // for angle
        (elbow_acceleration_x / fabs(elbow_acceleration_x)) *
        (elbow_acceleration_y / fabs(elbow_acceleration_y));  // for direction
    units::angle::degree_t home =
        static_cast<units::angle::degree_t>(rotunda_angle + elbow_angle);
    shoulder_.SetPosition(home);
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

  MissionControlData mc_data;

 private:
  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
  sjsu::arm::WristJoint & wrist_;
};
}  // namespace sjsu::arm
