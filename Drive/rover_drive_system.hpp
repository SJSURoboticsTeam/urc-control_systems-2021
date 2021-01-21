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
  /// @param mode Updates drive mode if new mode is different from current.
  /// @param rotation_angle adjusts wheel position depending on mode.
  /// @param speed adjusts the movement speed of the rover.
  void Move(char mode, float rotation_angle, float speed)
  {
    if (mission_control_data_.is_operational)
    {
      if (mode != static_cast<char>(current_mode_))
      {
        SetMode(mode);
      }
      HandleRoverMovement(rotation_angle, speed);
      UpdateMissionControlData();
    }
  };

  ///  Sets the new mode for the rover. Will reduce rover movement speed to zero
  ///  before changing wheel mode & tire alignment
  void SetMode(char mode = 'S')
  {
    switch (mode)
    {
      case 'D':
      case 'd':
        current_mode_ = Mode::kDrive;
        SetWheelSpeed(kZeroSpeed);
        SetDriveMode();
        break;
      case 'S':
      case 's':
        current_mode_ = Mode::kSpin;
        SetWheelSpeed(kZeroSpeed);
        SetSpinMode();
        break;
      case 'T':
      case 't':
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
      HandleDriveMode(speed, angle);
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

  /// Parses data from mission control to be used for commanding rover
  void ParseMissionControlData(char * response)
  {
    // parse data somehow using sscanf
    sjsu::LogInfo("MISSION CONTROL RESPONSE:\n%s", response);

    mission_control_data_.is_operational = true;
    mission_control_data_.drive_mode     = 'S';
    mission_control_data_.rotation_angle = 10.0f;
    mission_control_data_.speed          = 30.0f;
  };

  /// Gets the speed and position/angle of each wheel on the rover
  bool GetRoverData()
  {
    sjsu::LogInfo("left wheel speed: %f", left_wheel_.GetSpeed());
    sjsu::LogInfo("left wheel position: %f", left_wheel_.GetPosition());
    sjsu::LogInfo("right wheel speed: %f", right_wheel_.GetSpeed());
    sjsu::LogInfo("right wheel position: %f", right_wheel_.GetPosition());
    sjsu::LogInfo("back wheel speed: %f", back_wheel_.GetSpeed());
    sjsu::LogInfo("back wheel position: %f", back_wheel_.GetPosition());
    return true;
  };

  /// Aligns rover wheels all in the same direction, facing foward
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

  /// Aligns rover wheels perpendicular to their legs using their homing mark
  void SetSpinMode()
  {
    left_wheel_.HomeWheel();
    right_wheel_.HomeWheel();
    back_wheel_.HomeWheel();
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
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

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    back_wheel_.SetPosition(angle);
    SetWheelSpeed(speed);
  };

  /// Handles spin mode. Adjusts only the speed (aka the spin direction)
  void HandleSpinMode(units::angular_velocity::revolutions_per_minute_t speed)
  {
    SetWheelSpeed(speed);
  };

  /// Handles translation mode. Adjusts all the wheels, keeping them parallel
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
};
}  // namespace sjsu::drive