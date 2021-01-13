#pragma once

#include "utility/log.hpp"
#include "utility/time.hpp"
#include "utility/units.hpp"
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
    rover_is_operational_ = enable;
    left_wheel_.Enable(enable);
    right_wheel_.Enable(enable);
    back_wheel_.Enable(enable);
    SetMode('S');
  }
  /// Main function for handling all the rover drive system functionality.
  void Move(char mode,
            units::angle::degree_t rotation_angle,
            units::angular_velocity::revolutions_per_minute_t speed)
  {
    if (rover_is_operational_)
    {
      // checks if the mode is different than what is currently running
      if (mode != static_cast<char>(current_mode_))
      {
        SetMode(mode);
      }
      HandleModeMovement(rotation_angle, speed);
      UpdateMissionControlData();
    }
  };

  bool rover_is_operational_ = false;

 private:
  ///  Sets the new mode for the rover. Will reduce rover movement speed to zero
  ///  before changing wheel mode & tire alignment
  void SetMode(char mode)
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
  /// Handles the rover movement depending on the mode
  void HandleModeMovement(
      units::angle::degree_t roatation_angle,
      units::angular_velocity::revolutions_per_minute_t speed)
  {
    if (current_mode_ == Mode::kDrive)
    {
      HandleDriveMode();
    }
    if (current_mode_ == Mode::kSpin)
    {
      HandleSpinMode();
    }
    if (current_mode_ == Mode::kTranslation)
    {
      HandleTranslationMode();
    }
  };
  /// Calculates position to set each of the rover wheels to initially
  void SetDriveMode();
  void SetSpinMode();
  void SetTranslationMode();
  /// Handles the movement for their respective modes.
  void HandleDriveMode();
  void HandleSpinMode();
  void HandleTranslationMode();
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
  /// Gets the speed of each hub motor and angle of each steer motor on the
  /// rover
  bool GetRoverData()
  {
    // Format should be parsable by the Raspberry Pi like JSON ? Ex:
    // {"left_wheel_speed": "x_rpm", "right_wheel_speed": "y_rpm", ...}
    // rover_data_ = "Some key:value data struct?";
    return true;
  };

  // string rover_data_ = "";
  sjsu::drive::Wheel & left_wheel_;
  sjsu::drive::Wheel & right_wheel_;
  sjsu::drive::Wheel & back_wheel_;
  sjsu::drive::RoverDriveSystem::Mode current_mode_ = Mode::kSpin;
  units::angular_velocity::revolutions_per_minute_t current_speed_   = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;
};
}  // namespace sjsu::drive
