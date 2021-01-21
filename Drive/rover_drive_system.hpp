#pragma once

#include "utility/log.hpp"
#include "utility/time.hpp"
#include "utility/units.hpp"
#include "utility/map.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
class RoverDriveSystem
{
 public:
  /// Rover drive modes
  enum class Mode : char
  {
    kDrive       = 'D',
    kSpin        = 'S',
    kTranslation = 'T'
  };

  struct MissionControlData
  {
    bool is_operational;
    char drive_mode;
    float rotation_angle;
    float speed;
  };

  RoverDriveSystem(sjsu::drive::Wheel & left_wheel,
                   sjsu::drive::Wheel & right_wheel,
                   sjsu::drive::Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  void Initialize()
  {
    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
  };

  void Enable(bool enable = true)
  {
    mission_control_data_.is_operational = true;
    left_wheel_.Enable(enable);
    right_wheel_.Enable(enable);
    back_wheel_.Enable(enable);
    SetMode();
  }

  // Retrieves commands for drive movement from Mission Control. Translates the
  // Response data into usable format for rover functionality. Returns true if
  // successful. Specifically drive mode, rotation angle/position to move, and
  // speed.
  bool GetMissionControlData()
  {
    // GET /drive {sjsu::drive::RoverDriveSystem::Mode mode,
    // units::angle::degree_t rotation_angle,
    // units::angular_velocity::revolutions_per_minute_t speed}
    if (true)  // localhost connection found
    {
      char response[] = {};
      ParseMissionControlData(response);
      Move(mission_control_data_.drive_mode,
           mission_control_data_.rotation_angle, mission_control_data_.speed);
    }
    return true;
  }

  /// Main function for handling all the rover drive system functionality.
  void Move(char mode, float rotation_angle, float speed)
  {
    if (mission_control_data_.is_operational)
    {
      // checks if the mode is different than what is currently running
      if (mode != static_cast<char>(current_mode_))
      {
        SetMode(mode);
      }
      HandleRoverMovement(rotation_angle, speed);
    }
  };

  ///  Sets the new mode for the rover. Will reduce rover movement speed to zero
  ///  before changing wheel mode & tire alignment
  void SetMode(char mode = 'S')
  {
    switch (mode)
    {
      case 'D':
        current_mode_ = Mode::kDrive;
        SetWheelSpeed(kZeroSpeed);
        SetDriveMode();
        break;
      case 'S':
        current_mode_ = Mode::kSpin;
        SetWheelSpeed(kZeroSpeed);
        SetSpinMode();
        break;
      case 'T':
        current_mode_ = Mode::kTranslation;
        SetWheelSpeed(kZeroSpeed);
        SetTranslationMode();
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Unable to assign drive mode!");
    };
  };

  // Resets all the wheels so the motors know their actual position. Returns
  // true if successful.
  bool Reset()
  {
    SetMode();
    return true;
  };

 private:
  /// Handles the rover movement depending on the mode
  void HandleRoverMovement(float roatation_angle, float wheel_speed)
  {
    units::angle::degree_t angle(roatation_angle);
    units::angular_velocity::revolutions_per_minute_t speed(wheel_speed);

    if (current_mode_ == Mode::kDrive)
    {
      HandleDriveMode();
    }
    if (current_mode_ == Mode::kSpin)
    {
      HandleSpinMode(speed);
    }
    if (current_mode_ == Mode::kTranslation)
    {
      HandleTranslationMode(speed, angle);
    }
    current_speed_ = speed;
  };

  /// Sets all wheels to the speed provided. SetSpeed() handles max/min speeds
  void SetWheelSpeed(units::angular_velocity::revolutions_per_minute_t speed)
  {
    left_wheel_.SetSpeed(speed);
    right_wheel_.SetSpeed(speed);
    back_wheel_.SetSpeed(speed);
  };

  /// Sends POST to Raspberry Pi endpoint with the new rover status updates
  void UpdateMissionControlData()
  {
    if (GetRoverData())
    {
      // POST, http://localhost:3000/drive/status, JSON.stringify(rover_data_)
    }
    else
    {
      sjsu::LogError("Unable to retrieve wheel data!");
    }
  };

  void ParseMissionControlData(char * response);

  /// Gets the speed of each hub motor and angle of each steer motor on the
  /// rover. Does not get data from Mission Control scanf
  bool GetRoverData()
  {
    // Format should be parsable by the Raspberry Pi like JSON ? Ex:
    // {"left_wheel_speed": "x_rpm", "right_wheel_speed": "y_rpm", ...}
    // rover_data_ = "Some key:value data struct?";
    return true;
  };

  /// Aligns rover wheels where all wheel start off facing same direction
  void SetDriveMode()
  {
    units::angle::degree_t left_wheel_angle  = -45_deg;
    units::angle::degree_t right_wheel_angle = -135_deg;
    units::angle::degree_t back_wheel_angle  = 90_deg;
    SetSpinMode();
    left_wheel_.SetPosition(left_wheel_angle);
    right_wheel_.SetPosition(right_wheel_angle);
    back_wheel_.SetPosition(back_wheel_angle);
  };

  void SetSpinMode()
  {
    left_wheel_.HomeWheel();
    right_wheel_.HomeWheel();
    back_wheel_.HomeWheel();
  };
  void SetTranslationMode()
  {
    units::angle::degree_t left_wheel_angle  = 45_deg;
    units::angle::degree_t right_wheel_angle = -45_deg;
    units::angle::degree_t back_wheel_angle  = -180_deg;
    SetSpinMode();
    left_wheel_.SetPosition(left_wheel_angle);
    right_wheel_.SetPosition(right_wheel_angle);
    back_wheel_.SetPosition(back_wheel_angle);
  };

  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    back_wheel_.SetPosition(angle);
    SetWheelSpeed(speed);
  };

  void HandleSpinMode(units::angular_velocity::revolutions_per_minute_t speed)
  {
    SetWheelSpeed(speed);
  };

  void HandleTranslationMode(
      units::angular_velocity::revolutions_per_minute_t speed,
      units::angle::degree_t angle)
  {
    left_wheel_.SetPosition(angle);
    right_wheel_.SetPosition(angle);
    back_wheel_.SetPosition(angle);
    SetWheelSpeed(speed);
  };

  sjsu::drive::Wheel & left_wheel_;
  sjsu::drive::Wheel & right_wheel_;
  sjsu::drive::Wheel & back_wheel_;
  MissionControlData mission_control_data_;
  sjsu::drive::RoverDriveSystem::Mode current_mode_ = Mode::kSpin;
  units::angular_velocity::revolutions_per_minute_t current_speed_   = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxSpeed  = 100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNSpeed = -100_rpm;
};
}  // namespace sjsu::drive
