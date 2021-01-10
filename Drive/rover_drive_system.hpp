#pragma once

#include <string>

#include "utility/log.hpp"
#include "utility/time.hpp"
#include "utility/units.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
class RoverDriveSystem
{
 public:
  // Rover drive modes
  enum class Mode : char
  {
    kDrive     = 'D',
    kSpin      = 'S',
    kTranslate = 'T'
  };

  RoverDriveSystem(sjsu::drive::Wheel left_wheel,
                   sjsu::drive::Wheel right_wheel,
                   sjsu::drive::Wheel back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel)
  {
  }

  void Initialize()
  {
    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
  };

  void Enable(bool enable = true)
  {
    is_operational_ = enable;
    left_wheel_.Enable(enable);
    right_wheel_.Enable(enable);
    back_wheel_.Enable(enable);
    SetMode('S');
  }

  void Move(char mode,
            units::angle::degree_t rotation_angle,
            units::angular_velocity::revolutions_per_minute_t speed)
  {
    if (mode != static_cast<char>(current_mode_))
    {
      SetMode(mode);
    }
    HandleModeMovement(rotation_angle, speed);
  };

  bool is_operational_ = false;

 private:
  // Is there a way to
  void SetMode(char mode)
  {
    switch (mode)
    {
      case 'D':
        current_mode_ = Mode::kDrive;
        SetAllWheelsSpeed(kZeroSpeed);
        SetDriveMode();
        break;
      case 'S':
        current_mode_ = Mode::kSpin;
        SetAllWheelsSpeed(kZeroSpeed);
        SetSpinMode();
        break;
      case 'T':
        current_mode_ = Mode::kTranslate;
        SetAllWheelsSpeed(kZeroSpeed);
        SetTranslateMode();
        break;
      default: SetAllWheelsSpeed(kZeroSpeed);
    };
  };

  void HandleModeMovement(
      units::angle::degree_t roatation_angle,
      units::angular_velocity::revolutions_per_minute_t speed){

  };
  // Calculates position to set each of the rover wheels
  void SetDriveMode();
  void SetSpinMode();
  void SetTranslateMode();

  void MoveDriveMode();
  void MoveSpinMode();
  void MoveTranslateMode();

  void SetAllWheelsSpeed(
      units::angular_velocity::revolutions_per_minute_t speed)
  {
    left_wheel_.SetSpeed(speed);
    right_wheel_.SetSpeed(speed);
    back_wheel_.SetSpeed(speed);
  };

  void GetWheelData(){
    // Would be cool to get this in JSON format if possible. Ex:
    // {"left_wheel_speed": "x_rpm", "right_wheel_speed": "y_rpm", ...}
    // Not sure of easy way to do that...

    // left_wheel_.GetSpeed().to<double>(); // lookup .to<string> ?
    // left_wheel_.GetPosition();
  };

  sjsu::drive::Wheel & left_wheel_;
  sjsu::drive::Wheel & right_wheel_;
  sjsu::drive::Wheel & back_wheel_;
  sjsu::drive::RoverDriveSystem::Mode current_mode_ = Mode::kSpin;
  units::angular_velocity::revolutions_per_minute_t current_speed_   = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;
};
}  // namespace sjsu::drive