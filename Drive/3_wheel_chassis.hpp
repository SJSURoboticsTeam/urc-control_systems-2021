#pragma once

#include <array>

#include "utility/log.hpp"

#include "Interface/chassis.hpp"
#include "Interface/wheel.hpp"

namespace sjsu::drive
{
class ThreeWheeledChassis : public Chassis
{
  struct ParseError
  {
  };
  struct DriveModeError
  {
  };
  struct DriveModeHandlerError
  {
  };

  enum class Modes : char
  {
    DriveMode      = 'D',
    SpinMode       = 'S',
    TranslateMode  = 'T',
    LeftWheelMode  = 'L',
    RightWheelMode = 'R',
    BackWheelMode  = 'B'
  };

  ThreeWheeledChassis(Wheel & left_wheel,
                      Wheel & right_wheel,
                      Wheel & back_wheel)
      : left_wheel_(&left_wheel),
        right_wheel_(&right_wheel),
        back_wheel_(&back_wheel){};

  void Initialize() override
  {
    left_wheel_->Initialize();
    right_wheel_->Initialize();
    back_wheel_->Initialize();
    SetSpinMode();
  }

  /// 0 = Left/Right/Back, 1 = Back/Left/Right, 2 = Right/Back/Left
  void SwitchLegOrientation(int position)
  {
    if (IsStopped())
    {
      left_wheel_  = wheels_[(position + 0) % 3];
      right_wheel_ = wheels_[(position + 1) % 3];
      back_wheel_  = wheels_[(position + 2) % 3];
    }
  };

  void HandleMovement(Modes drive_mode_, float angle, float speed)
  {
    switch (drive_mode_)
    {
      case Modes::DriveMode: HandleDriveMode(speed, angle); break;
      case Modes::SpinMode: HandleSpinMode(speed); break;
      case Modes::TranslateMode: HandleTranslationMode(speed, angle); break;
      case Modes::LeftWheelMode:
      case Modes::RightWheelMode:
      case Modes::BackWheelMode: HandleSingularWheelMode(speed, angle); break;
      default: StopWheels(); break;  // throw DriveModeHandlerError{};
    }
  }

  Modes GetCurrentMode()
  {
    return current_drive_mode_;
  }

  /// Locks the thread until all wheels are stopped
  void StopWheels()
  {
    sjsu::LogInfo("Stopping rover...");
    while (!IsStopped())
    {
      SetWheelSpeed(kZeroSpeed);
    }
  }

  /// Checks if the rover wheels are all stopped
  bool IsStopped()
  {
    return (left_wheel_->GetHubSpeed() == 0 &&
            right_wheel_->GetHubSpeed() == 0 &&
            back_wheel_->GetHubSpeed() == 0);
  }

  /// Slowly lerps the wheels toward the target speed
  void SetWheelSpeed(float target_speed)
  {
    float left_wheel_speed  = float(left_wheel_->GetHubSpeed());
    float right_wheel_speed = float(right_wheel_->GetHubSpeed());
    float back_wheel_speed  = float(back_wheel_->GetHubSpeed());

    left_wheel_speed  = std::lerp(left_wheel_speed, target_speed, kLerpStep);
    right_wheel_speed = std::lerp(right_wheel_speed, target_speed, kLerpStep);
    back_wheel_speed  = std::lerp(back_wheel_speed, target_speed, kLerpStep);

    left_wheel_->SetHubSpeed(left_wheel_speed);
    right_wheel_->SetHubSpeed(right_wheel_speed);
    back_wheel_->SetHubSpeed(back_wheel_speed);
  }

  /// Locks thread until all wheels are homed
  void HomeWheels()
  {
    StopWheels();
    sjsu::LogInfo("Homing the wheels...");
    // Setting wheels to zero (normally angle) until slip ring gets fixed
    for (int angle = 0; angle < 360; angle += 2)
    {
      if (AllWheelsAreHomed())
      {
        break;
      }
      if (!left_wheel_->IsHomed())
      {
        left_wheel_->SetSteerAngle(0);
      }
      if (!right_wheel_->IsHomed())
      {
        right_wheel_->SetSteerAngle(0);
      }
      if (!back_wheel_->IsHomed())
      {
        back_wheel_->SetSteerAngle(0);
      }
      sjsu::Delay(50ms);
    }
    sjsu::LogInfo("Wheels homed!");
  }

  bool AllWheelsAreHomed()
  {
    return (left_wheel_->IsHomed() && right_wheel_->IsHomed() &&
            back_wheel_->IsHomed());
  }

 private:
  /// Stops the rover and sets a new mode.
  void SetMode()
  {
    sjsu::LogWarning("Switching rover into %c mode...", mc_data_.drive_mode);
    StopWheels();
    switch (mc_data_.drive_mode)
    {
      case Modes::DriveMode: SetDriveMode(); break;
      case Modes::SpinMode: SetSpinMode(); break;
      case Modes::TranslateMode: SetTranslationMode(); break;
      case Modes::LeftWheelMode:
      case Modes::RightWheelMode:
      case Modes::BackWheelMode: SetSingleWheelMode(); break;
      default: break;  // throw DriveModeError{};
    };
  }

  /// Aligns rover wheels all in the same direction, facing forward
  void SetDriveMode()
  {
    const int left_wheel_angle  = -45;
    const int right_wheel_angle = -135;
    const int back_wheel_angle  = 90;
    left_wheel_->SetSteerAngle(left_wheel_angle);
    right_wheel_->SetSteerAngle(right_wheel_angle);
    back_wheel_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::DriveMode;
  }

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    const float left_wheel_angle  = 90;
    const float right_wheel_angle = 90;
    const float back_wheel_angle  = 90;
    left_wheel_->SetSteerAngle(left_wheel_angle);
    right_wheel_->SetSteerAngle(right_wheel_angle);
    back_wheel_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::SpinMode;
  }

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    // TODO: Find the angles for translation mode
    const float left_wheel_angle  = 0;
    const float right_wheel_angle = 60;
    const float back_wheel_angle  = 110;
    left_wheel_->SetSteerAngle(left_wheel_angle);
    right_wheel_->SetSteerAngle(right_wheel_angle);
    back_wheel_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::TranslateMode;
  }

  void SetSingleWheelMode()
  {
    current_drive_mode_ = mc_data_.drive_mode;
  }

  void HandleDriveMode(float speed, float inner_wheel_angle)
  {
    inner_wheel_angle =
        std::clamp(inner_wheel_angle, -kMaxTurnRadius, kMaxTurnRadius);

    float outter_wheel_angle(GetOutterWheelDriveAngle(inner_wheel_angle));
    float back_wheel_angle(GetBackWheelDriveAngle(inner_wheel_angle));

    if (inner_wheel_angle > 0)
    {
      right_wheel_->SetSteerAngle(inner_wheel_angle);
      left_wheel_->SetSteerAngle(outter_wheel_angle);
      back_wheel_->SetSteerAngle(back_wheel_angle);
    }
    else
    {
      right_wheel_->SetSteerAngle(outter_wheel_angle);
      left_wheel_->SetSteerAngle(inner_wheel_angle);
      back_wheel_->SetSteerAngle(back_wheel_angle);
    }
    // TODO: Need logic for controling wheel speed for each wheel
    SetWheelSpeed(speed);
  }

  /// Calculates outer wheel angle based off inner wheel angle
  float GetOutterWheelDriveAngle(float inner_wheel_angle)
  {
    float outter_wheel_angle = 0.392 + 0.744 * abs(int(inner_wheel_angle)) +
                               -0.0187 * pow(abs(int(inner_wheel_angle)), 2) +
                               1.84E-04 * pow(abs(int(inner_wheel_angle)), 3);
    return (inner_wheel_angle > 0) ? outter_wheel_angle : -outter_wheel_angle;
  }

  /// Calculates back wheel angle based off inner wheel angle
  float GetBackWheelDriveAngle(float inner_wheel_angle)
  {
    float back_wheel_angle = -0.378 + -1.79 * abs(int(inner_wheel_angle)) +
                             0.0366 * pow(abs(int(inner_wheel_angle)), 2) +
                             -3.24E-04 * pow(abs(int(inner_wheel_angle)), 3);
    return (inner_wheel_angle > 0) ? back_wheel_angle : -back_wheel_angle;
  }

  /// Adjusts only the hub speed since rover will spin in place
  void HandleSpinMode(float speed)
  {
    SetWheelSpeed(speed);
  }

  /// Adjusts all the wheels by keeping them in parallel
  void HandleTranslationMode(float speed, float angle)
  {
    // TODO: Need to find correct angles
    left_wheel_->SetSteerAngle(angle);
    right_wheel_->SetSteerAngle(angle);
    back_wheel_->SetSteerAngle(angle);
    SetWheelSpeed(speed);
  }

  /// Adjusts the hub speed and steer angle of the specified wheel
  void HandleSingularWheelMode(float speed, float angle)
  {
    switch (current_drive_mode_)
    {
      case Modes::LeftWheelMode:
        left_wheel_->SetSteerAngle(angle);
        left_wheel_->SetHubSpeed(speed);
        break;
      case Modes::RightWheelMode:
        right_wheel_->SetSteerAngle(angle);
        right_wheel_->SetHubSpeed(speed);
        break;
      case Modes::BackWheelMode:
        back_wheel_->SetSteerAngle(angle);
        back_wheel_->SetHubSpeed(speed);
        break;
      default: break;
    }
  }

  Wheel * left_wheel_;
  Wheel * right_wheel_;
  Wheel * back_wheel_;
  std::array<Wheel *, 3> wheels_{ left_wheel_, right_wheel_, back_wheel_ };
};
}  // namespace sjsu::drive