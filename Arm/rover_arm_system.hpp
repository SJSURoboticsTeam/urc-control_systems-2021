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
    std::string parameter =
        "?is_operational=" + std::to_string(mc_data_.is_operational) +
        "&heartbeat_count=" + std::to_string(heartbeat_count_);
    return parameter;
  };

  std::string ParseJSONResponse()
  {
    return "Fill";
  };

  bool SyncedWithMissionControl()
  {
    bool fill;
    return fill;
  };

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

    VerifyNonZeroes(accelerations_.rotunda.x);
    VerifyNonZeroes(accelerations_.rotunda.y);
    VerifyNonZeroes(accelerations_.shoulder.x);
    VerifyNonZeroes(accelerations_.shoulder.y);

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
    MoveShoulder(home);  // move shoulder at 10 rpm to home
  }
  // logic needs checking.

  void HomeElbow()
  {
    double home = 0;
    // VerifyNonZeroes()
    VerifyNonZeroes(accelerations_.rotunda.x);
    VerifyNonZeroes(accelerations_.rotunda.y);
    VerifyNonZeroes(accelerations_.elbow.x);
    VerifyNonZeroes(accelerations_.elbow.y);

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
    MoveElbow(home);
  }

  void HomeWrist(){};

  void Esp(){};

  // TODO: need to work on valid movement durring in person work shop
  bool CheckValidMovement()
  {
    bool fill;
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
  sjsu::arm::Joint & rotunda_;
  sjsu::arm::Joint & shoulder_;
  sjsu::arm::Joint & elbow_;
  sjsu::arm::WristJoint & wrist_;
  MissionControlData mc_data_;
  Acceleration accelerations_;
  int heartbeat_count_                    = 0;
  MissionControlData::Modes current_mode_ = MissionControlData::Modes::kDefault;

  void VerifyNonZeroes(double & acceleration)
  {
    if (acceleration == 0)
    {
      acceleration = 0.0001;
    }
  }

  double FindComplimentValue(
      double,
      double){};  // may or may not need, will decide after testing code
};
}  // namespace sjsu::arm
