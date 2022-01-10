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
    Modes modes         = Modes::kDefault;
    int is_operational  = 1;
    int heartbeat_count = 0;
    double arm_speed;
    double rotunda_angle;
    double shoulder_angle;
    double elbow_angle;
    double wrist_roll;
    double wrist_pitch;

    struct Finger
    {
      double pinky_angle;
      double ring_angle;
      double middle_angle;
      double pointer_angle;
      double thumb_angle;
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
                 sjsu::arm::WristJoint wrist)
      : rotunda_(rotunda), shoulder_(shoulder), elbow_(elbow), wrist_(wrist)
  {
  }
  void Initialize()
  {
    rotunda_.Initialize();  // returns
    shoulder_.Initialize();
    elbow_.Initialize();
    wrist_.Initialize();
  }

  void PrintRoverData(){

  };

  std::string GETParameters()
  {
    char request_parameter[300];
    snprintf(request_parameter, 300,
             "?heartbeat_count=%d&is_operational=%d&arm_speed=%c&battery=%d"
             "&rotunda_angle=%d&shoulder_angle=%d&elbow_angle=%d&wrist_roll=%d"
             "&back_wheel_speed=%d&wrist_pitch=%d&pinky_angle=%d&ring_angle=%d"
             "&middle_angle=%d&pointer_angle=%d&thumb_angle=%d",
             heartbeat_count_, mc_data_.is_operational, mc_data_.arm_speed,
             state_of_charge_, rotunda_.GetPosition(), shoulder_.GetPosition(),
             elbow_.GetPosition(), wrist_.GetRollPosition(),
             wrist_.GetPitchPosition(), hand_.GetPinkyPosition(),
             hand_.GetRingPosition(), hand_.GetMiddlePosition(),
             hand_.GetPointerPosition(), hand_.GetThumbPosition());

    return request_parameter;
  };

  void ParseJSONResponse(std::string & response)
  {
    int actual_arguments = sscanf(
        response.c_str(), response_body_format, &mc_data_.heartbeat_count,
        &mc_data_.is_operational, &mc_data_.arm_speed, &mc_data_.rotunda_angle,
        &mc_data_.shoulder_angle, &mc_data_.elbow_angle, &mc_data_.wrist_roll,
        &mc_data_.wrist_pitch, &mc_data_.finger.pinky_angle,
        &mc_data_.finger.ring_angle, &mc_data_.finger.middle_angle,
        &mc_data_.finger.pointer_angle, &mc_data_.finger.thumb_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  };

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

  /// Verifies that mission control is sending fresh commands to rover
  bool IsHeartbeatSynced()
  {
    if (mc_data_.heartbeat_count != heartbeat_count_)
    {
      // TODO: Throw error if this is reached?
      sjsu::LogError("Heartbeat out of sync - resetting!");
      heartbeat_count_ = 0;
      return false;
    }
    return true;
  }

  void IncrementHeartbeatCount()
  {
    heartbeat_count_++;
  }

  void MoveRotunda(double angle)
  {
    rotunda_.SetSpeed(mc_data_.arm_speed);
    rotunda_.SetPosition(angle);
  }

  void MoveShoulder(double angle)
  {
    shoulder_.SetSpeed(mc_data_.arm_speed);
    shoulder_.SetPosition(angle);
  }

  void MoveElbow(double angle)
  {
    elbow_.SetSpeed(mc_data_.arm_speed);
    elbow_.SetPosition(angle);
  }

  void HandleArmMovement()
  {
    // TODO: implement different arm drive modes in this function.

    // D drive mode
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

    HomeWrist();
  }

  void HomeShoulder()
  {
    double home = 0;
    // This checks to see if any of the
    // acceleration values are 0, and if they are, it will change
    // to something close to 0

    ChangeIfZero(accelerations_.rotunda.x);
    ChangeIfZero(accelerations_.rotunda.y);
    ChangeIfZero(accelerations_.shoulder.x);
    ChangeIfZero(accelerations_.shoulder.y);

    // cut this out into a helper function to clean up code
    // double check logic: add the values together first then calculate the
    // angle needed

    double acceleration_x =
        accelerations_.rotunda.x +
        accelerations_.shoulder.x;  // might need compliment value of shoulder
    double acceleration_y =
        accelerations_.rotunda.y + accelerations_.shoulder.y;
    // adding i and j vectors of acceleration
    home = atan(acceleration_y / acceleration_x);
    shoulder_.SetZeroOffset(home);
    MoveShoulder(home);  // move shoulder at 10 rpm to home
  }
  // logic needs checking.

  void HomeElbow()
  {
    double home = 0;
    // ChangeIfZero()
    ChangeIfZero(accelerations_.rotunda.x);
    ChangeIfZero(accelerations_.rotunda.y);
    ChangeIfZero(accelerations_.elbow.x);
    ChangeIfZero(accelerations_.elbow.y);

    double acceleration_x =
        accelerations_.rotunda.x +
        accelerations_.elbow.x;  // might need compliment value of shoulder
    double acceleration_y = accelerations_.rotunda.y + accelerations_.elbow.y;
    double angle_without_correction = atan(acceleration_y / acceleration_x);
    // maybe make the following statements above the if's functions that return
    // a bool to give a better description/make it look nicer if the elbow is in
    // the second quadrant of a graph, add 90 to the angle
    if (accelerations_.elbow.x >= 0 && accelerations_.elbow.y <= 0)
    {
      home = 90 - angle_without_correction;
    }
    // if the elbow is in the third quadrant of a graph, add 180 to the angle
    else if (accelerations_.elbow.x >= 0 && accelerations_.elbow.y >= 0)
    {
      home = 180 + angle_without_correction;
    }
    else
    {
      home = angle_without_correction;
    }
    elbow_.SetZeroOffset(home);
    MoveElbow(home);
  }

  void HomeWrist(){};

  void Esp(){};

  // TODO: need to work on valid movement durring in person work shop
  bool CheckValidMovement()
  {
    bool fill = true;
    return fill;
  }

  void Calibrate(){};

  void SendUpdateToMC(){};

  void PrintArmMode(){};

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
  void ChangeIfZero(double & acceleration)
  {
    if (acceleration == 0)
    {
      acceleration = 0.0001;
    }
  }
  double FindComplimentValue(){};  // may or may not need, will decide after
                                   // testing code

  Acceleration accelerations_;
  int heartbeat_count_                    = 0;
  int state_of_charge_                    = 90;
  const int kExpectedArguments            = 13;
  MissionControlData::Modes current_mode_ = MissionControlData::Modes::kDefault;

 public:
  sjsu::arm::Hand hand_;
  MissionControlData mc_data_;
  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
};
}  // namespace sjsu::arm