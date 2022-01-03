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

  void PrintRoverData(){};

  std::string GETParameters()
  {
    return "Fill";
  };

  std::string ParseJSONResponse()
  {
    return "Fill";
  };

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
    // wrist_roll(static_cast<float>(mc_data.wrist_roll)); units::angle::degree_t
    // wrist_pitch(static_cast<float>(mc_data.wrist_pitch));

    // TODO: implement different arm drive modes in this function.

    // D drive mode
    rotunda_.SetSpeed(rotunda_speed);
    rotunda_.SetPosition(rotunda_angle);
    shoulder_.SetSpeed(shoulder_speed);
    shoulder_.SetPosition(shoulder_angle);
    elbow_.SetSpeed(elbow_speed);
    elbow_.SetPosition(elbow_angle);
  }

  void HomeArm()
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

  private:
  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
  sjsu::arm::WristJoint & wrist_;

};
}  // namespace sjsu::arm
