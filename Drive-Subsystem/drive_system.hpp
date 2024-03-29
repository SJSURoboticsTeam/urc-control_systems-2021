#pragma once
#include <array>

#include "wheel.hpp"
#include "utility/log.hpp"
#include "../Common/state_of_charge.hpp"
#include "../Common/Interface/rover_system_interface.hpp"

namespace sjsu::drive
{
const char response_body_format[] =
    "\r\n\r\n{\n"
    "  \"heartbeat_count\": %d,\n"
    "  \"is_operational\": %d,\n"
    "  \"wheel_shift\": %d,\n"
    "  \"drive_mode\": \"%c\",\n"
    "  \"speed\": %d,\n"
    "  \"angle\": %d\n"
    "}";
class RoverDriveSystem : public sjsu::common::RoverSystemInterface
{
 public:
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

  struct Wheels
  {
    Wheel * left_  = nullptr;
    Wheel * right_ = nullptr;
    Wheel * back_  = nullptr;
  };

  struct MissionControlData : public RoverMissionControlData
  {
    int wheel_shift    = 0;
    int rotation_angle = 0;
    int speed          = 0;
    Modes drive_mode   = Modes::SpinMode;
  };

  RoverDriveSystem(Wheels & wheels) : wheels_(wheels){};

  void Initialize() override
  {
    sjsu::LogInfo("Initializing drive system...");
    wheels_.left_->Initialize();
    wheels_.right_->Initialize();
    wheels_.back_->Initialize();
    // SetSpinMode();
    sjsu::LogInfo("Drive system initialized!");
  }
  /// [0] = {L, R, B}, [1] = {B, L, R}, [2] = {R, B, L}
  void SwitchLegOrientation(int position)
  {
    if (IsStopped() && position >= 0)
    {
      wheels_.left_  = wheel_array_[(position + 0) % 3];
      wheels_.right_ = wheel_array_[(position + 1) % 3];
      wheels_.back_  = wheel_array_[(position + 2) % 3];
    }
  };

  /// Constructs parameters for an HTTP GET request
  /// @return ?heartbeat_count=0&is_operational=1&drive_mode=S ...
  std::string CreateGETRequestParameterWithRoverStatus() override
  {
    char request_parameter[300];
    snprintf(
        request_parameter, 300,
        "?heartbeat_count=%d&is_operational=%d&wheel_shift=%d&drive_mode=%c&"
        "battery=%d&left_wheel_speed=%d&left_wheel_angle=%d&right_wheel_speed=%"
        "d&right_wheel_angle=%d&back_wheel_speed=%d&back_wheel_angle=%d",
        heartbeat_.GetHeartbeatCount(), mc_data_.is_operational,
        mc_data_.wheel_shift, static_cast<char>(current_drive_mode_),
        static_cast<int>(state_of_charge_.StateOfCharge_MAX()),
        wheels_.left_->GetHubSpeed(), wheels_.left_->GetSteerAngle(),
        wheels_.right_->GetHubSpeed(), wheels_.right_->GetSteerAngle(),
        wheels_.back_->GetHubSpeed(), wheels_.back_->GetSteerAngle());
    return request_parameter;
  }
  /// Parses the GET requests response and updates the mission control variables
  void ParseMissionControlCommands(std::string & response) override
  {
    char drive_mode;
    int actual_arguments = sscanf(
        response.c_str(), response_body_format, &mc_data_.heartbeat_count,
        &mc_data_.is_operational, &mc_data_.wheel_shift, &drive_mode,
        &mc_data_.speed, &mc_data_.rotation_angle);

    mc_data_.drive_mode = Modes{ drive_mode };

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }

  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation, L/R/B = Left/Right/Back Wheel
  void HandleRoverCommands() override
  {
    if (!IsSyncedWithMissionControl(mc_data_.heartbeat_count))
    {
      SetWheelSpeed(kZeroSpeed, mc_data_.rotation_angle);
      return;
    }
    if (!IsOperational(mc_data_.is_operational))
    {
      StopWheels();
      return;
    }
    if (IsNewMode())
    {
      SetMode();
      return;
    }

    float angle = float(mc_data_.rotation_angle);
    float speed = float(mc_data_.speed);

    switch (current_drive_mode_)
    {
      case Modes::DriveMode: HandleDriveMode(speed, angle); break;
      case Modes::SpinMode: HandleSpinMode(speed); break;
      case Modes::TranslateMode: HandleTranslationMode(speed, angle); break;
      case Modes::LeftWheelMode:
      case Modes::RightWheelMode:
      case Modes::BackWheelMode: HandleSingularWheelMode(speed, angle); break;
      default:
        StopWheels();
        // throw DriveModeHandlerError{};
        break;
    }
  }

  /// Checks if the rover got a new drive mode command
  bool IsNewMode()
  {
    if (mc_data_.drive_mode != current_drive_mode_)
    {
      sjsu::LogWarning("Rover was assigned new drive mode!");
      return true;
    }
    return false;
  }

  Modes GetCurrentMode() const
  {
    return current_drive_mode_;
  }

  Wheels GetWheels()
  {
    return wheels_;
  }

  /// Locks the thread until all wheels are stopped
  void StopWheels()
  {
    sjsu::LogInfo("Stopping rover...");
    while (!IsStopped())
    {
      SetWheelSpeed(kZeroSpeed, mc_data_.rotation_angle);
    }
  }

  /// Checks if the rover wheels are all stopped
  bool IsStopped()
  {
    return (wheels_.left_->GetHubSpeed() == 0 &&
            wheels_.right_->GetHubSpeed() == 0 &&
            wheels_.back_->GetHubSpeed() == 0);
  }

  /// Slowly lerps the wheels toward the target speed
  void SetWheelSpeed(float target_speed, float inner_wheel_angle)
  {
    float left_wheel_speed  = float(wheels_.left_->GetHubSpeed());
    float right_wheel_speed = float(wheels_.right_->GetHubSpeed());
    float back_wheel_speed  = float(wheels_.back_->GetHubSpeed());

    if (inner_wheel_angle > 0)
    {
      left_wheel_speed =
          std::lerp(left_wheel_speed,
                    -(GetOutterWheelHubSpeed(target_speed, inner_wheel_angle)),
                    kLerpStep);
      right_wheel_speed = std::lerp(
          right_wheel_speed,
          -(GetInnerWheelHubSpeed(target_speed, inner_wheel_angle)), kLerpStep);
      back_wheel_speed = std::lerp(
          back_wheel_speed,
          GetBackWheelHubSpeed(target_speed, inner_wheel_angle), kLerpStep);
    }

    if (inner_wheel_angle < 0)
    {
      left_wheel_speed = std::lerp(
          left_wheel_speed,
          -(GetInnerWheelHubSpeed(target_speed, inner_wheel_angle)), kLerpStep);
      right_wheel_speed =
          std::lerp(right_wheel_speed,
                    -(GetOutterWheelHubSpeed(target_speed, inner_wheel_angle)),
                    kLerpStep);
      back_wheel_speed = std::lerp(
          back_wheel_speed,
          GetBackWheelHubSpeed(target_speed, inner_wheel_angle), kLerpStep);
    }
    if (inner_wheel_angle == 0)
    {
      left_wheel_speed = std::lerp(left_wheel_speed, -target_speed, kLerpStep);
      right_wheel_speed =
          std::lerp(right_wheel_speed, -target_speed, kLerpStep);
      back_wheel_speed = std::lerp(back_wheel_speed, target_speed, kLerpStep);
    }

    wheels_.left_->SetHubSpeed(left_wheel_speed);
    wheels_.right_->SetHubSpeed(right_wheel_speed);
    wheels_.back_->SetHubSpeed(back_wheel_speed);
  }

  float GetInnerWheelHubSpeed(float inner_wheel_speed, float inner_wheel_angle)
  {
    // clamps the inner wheel speed to be no faster then what will mess up the
    // correct ackerman velocity this clamp will then ensure the same for the
    // back wheel speed since its based on this angle
    float ratio = GetOutterWheelRadius(inner_wheel_angle) /
                  GetInnerWheelRadius(inner_wheel_angle);
    std::clamp(inner_wheel_speed, -100 / ratio, 100 / ratio);
    return inner_wheel_speed;
  }

  float GetBackWheelHubSpeed(float inner_wheel_speed, float inner_wheel_angle)
  {
    float ratio = GetBackWheelRadius(inner_wheel_angle) /
                  GetInnerWheelRadius(inner_wheel_angle);
    float back_wheel_speed =
        std::clamp(inner_wheel_speed * ratio, -100 / ratio, 100 / ratio);
    return back_wheel_speed;
  }

  float GetOutterWheelHubSpeed(float inner_wheel_speed, float inner_wheel_angle)
  {
    float ratio = GetOutterWheelRadius(inner_wheel_angle) /
                  GetInnerWheelRadius(inner_wheel_angle);
    float outter_wheel_speed = inner_wheel_speed * ratio;
    return outter_wheel_speed;
  }

  float GetInnerWheelRadius(float inner_wheel_angle)
  {
    float inner_wheel_radius = 15 * pow(abs(inner_wheel_angle), -.971);
    return inner_wheel_radius;
  }

  float GetBackWheelRadius(float inner_wheel_angle)
  {
    float back_wheel_radius = 11.6 * pow(abs(inner_wheel_angle), -.698);
    return back_wheel_radius;
  }

  float GetOutterWheelRadius(float inner_wheel_angle)
  {
    float outter_wheel_radius = 11.6 * pow(abs(inner_wheel_angle), -.616);
    return outter_wheel_radius;
  }

  /// Locks thread until all wheels are homed
  void HomeWheels()
  {
    // StopWheels();
    sjsu::LogInfo("Homing the wheels...");

    wheels_.back_->SetHomingOffset(60.0 *
                                   (wheels_.back_->GetSteerMotor()
                                        .RequestFeedbackFromMotor()
                                        .GetFeedback()
                                        .encoder_position >>
                                    8) /
                                   256);
    wheels_.left_->SetHomingOffset(60.0 *
                                   (wheels_.left_->GetSteerMotor()
                                        .RequestFeedbackFromMotor()
                                        .GetFeedback()
                                        .encoder_position >>
                                    8) /
                                   256);
    wheels_.right_->SetHomingOffset(60.0 *
                                    (wheels_.right_->GetSteerMotor()
                                         .RequestFeedbackFromMotor()
                                         .GetFeedback()
                                         .encoder_position >>
                                     8) /
                                    256);
    wheels_.back_->SetSteerAngle(0);
    wheels_.left_->SetSteerAngle(0);
    wheels_.right_->SetSteerAngle(0);
    SetDriveMode();
    sjsu::LogInfo("Drive system is homed!");
  }

  bool AllWheelsAreHomed()
  {
    return (wheels_.left_->IsHomed() && wheels_.right_->IsHomed() &&
            wheels_.back_->IsHomed());
  }

  /// Prints the mc data and all the current wheel data
  void PrintRoverData() override
  {
    printf("HEARTBEAT:\t%d\n", mc_data_.heartbeat_count);
    printf("OPERATIONAL:\t%d\n", mc_data_.is_operational);
    printf("WHEEL SHIFT:\t%d\n", mc_data_.wheel_shift);
    printf("DRIVE MODE:\t%c\n", static_cast<char>(current_drive_mode_));
    printf("MC SPEED:\t%d\n", mc_data_.speed);
    printf("MC ANGLE:\t%d\n", mc_data_.rotation_angle);
    printf("WHEEL     SPEED     ANGLE\n");
    printf("=========================================\n");
    wheels_.left_->Print();
    wheels_.right_->Print();
    wheels_.back_->Print();
    printf("=========================================\n");
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
      default:
        // throw DriveModeError{};
        break;
    };
  }

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing forward
  void SetDriveMode()
  {
    const int left_wheel_angle = (-wheels_.left_->GetHomingOffset()) + 62.3;
    const int right_wheel_angle =
        -(wheels_.right_->GetHomingOffset() + 120 - 11);
    const int back_wheel_angle = 0;
    wheels_.left_->SetSteerAngle(left_wheel_angle);
    wheels_.right_->SetSteerAngle(right_wheel_angle);
    wheels_.back_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::DriveMode;
  }

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    const float left_wheel_angle  = -120;
    const float right_wheel_angle = -120.5;
    const float back_wheel_angle  = 120;
    wheels_.left_->SetSteerAngle(left_wheel_angle);
    wheels_.right_->SetSteerAngle(right_wheel_angle);
    wheels_.back_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::SpinMode;
  }

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  // do we want to set up multiple translate modes
  {
    // TODO: Find the angles for translation mode
    const float left_wheel_angle  = 0;
    const float right_wheel_angle = 60;
    const float back_wheel_angle  = 110;
    wheels_.left_->SetSteerAngle(left_wheel_angle);
    wheels_.right_->SetSteerAngle(right_wheel_angle);
    wheels_.back_->SetSteerAngle(back_wheel_angle);
    current_drive_mode_ = Modes::TranslateMode;
  }

  void SetSingleWheelMode()
  {
    current_drive_mode_ = mc_data_.drive_mode;
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  void HandleDriveMode(float speed, float inner_wheel_angle)
  {
    const int left_wheel_angle = (-wheels_.left_->GetHomingOffset()) + 62.3;
    const int right_wheel_angle =
        -(wheels_.right_->GetHomingOffset() + 120 - 11);
    inner_wheel_angle =
        std::clamp(inner_wheel_angle, -kMaxTurnRadius, kMaxTurnRadius);

    float outter_wheel_angle(GetOutterWheelDriveAngle(inner_wheel_angle));
    float back_wheel_angle(GetBackWheelDriveAngle(inner_wheel_angle));

    if (inner_wheel_angle > 0)
    {
      wheels_.right_->SetSteerAngle(inner_wheel_angle + right_wheel_angle);
      wheels_.left_->SetSteerAngle(outter_wheel_angle + left_wheel_angle);
      wheels_.back_->SetSteerAngle(back_wheel_angle);
    }
    else
    {
      wheels_.right_->SetSteerAngle(outter_wheel_angle + right_wheel_angle);
      wheels_.left_->SetSteerAngle(inner_wheel_angle + left_wheel_angle);
      wheels_.back_->SetSteerAngle(back_wheel_angle);
    }
    // TODO: Need logic for controling wheel speed for each wheel
    SetWheelSpeed(speed, inner_wheel_angle);
  }

  void GetLeftWheelSpeed(float speed, float inner_wheel_angle) {}

  void GetRightWheelSpeed(float speed, float inner_wheel_angle) {}

  void GetBackWheelSpeed(float speed, float inner_wheel_angle) {}

  /// Calculates outer wheel angle based off inner wheel angle
  float GetOutterWheelDriveAngle(float inner_wheel_angle) const
  {
    float outter_wheel_angle =
        float(0.392 + 0.744 * abs(int(inner_wheel_angle)) +
              -0.0187 * pow(abs(int(inner_wheel_angle)), 2) +
              1.84E-04 * pow(abs(int(inner_wheel_angle)), 3));
    return (inner_wheel_angle > 0) ? outter_wheel_angle : -outter_wheel_angle;
  }

  /// Calculates back wheel angle based off inner wheel angle
  float GetBackWheelDriveAngle(float inner_wheel_angle) const
  {
    float back_wheel_angle =
        float(-0.378 + -1.79 * abs(int(inner_wheel_angle)) +
              0.0366 * pow(abs(int(inner_wheel_angle)), 2) +
              -3.24E-04 * pow(abs(int(inner_wheel_angle)), 3));
    return (inner_wheel_angle > 0) ? back_wheel_angle : -back_wheel_angle;
  }

  /// Adjusts only the hub speed since rover will spin in place
  void HandleSpinMode(float speed)
  {
    SetWheelSpeed(speed, mc_data_.rotation_angle);
  }

  /// Adjusts all the wheels by keeping them in parallel
  void HandleTranslationMode(
      float speed,
      float angle)  // precision may be off, we will have to see
  {
    // TODO: Need to find correct angles
    wheels_.left_->SetSteerAngle(angle);
    wheels_.right_->SetSteerAngle(angle);
    wheels_.back_->SetSteerAngle(angle);
    SetWheelSpeed(speed, mc_data_.rotation_angle);
  }

  /// Adjusts the hub speed and steer angle of the specified wheel
  void HandleSingularWheelMode(float speed, float angle)
  {
    switch (current_drive_mode_)
    {
      case Modes::LeftWheelMode:
        wheels_.left_->SetSteerAngle(angle);
        wheels_.left_->SetHubSpeed(speed);
        break;
      case Modes::RightWheelMode:
        wheels_.right_->SetSteerAngle(angle);
        wheels_.right_->SetHubSpeed(speed);
        break;
      case Modes::BackWheelMode:
        wheels_.back_->SetSteerAngle(angle);
        wheels_.back_->SetHubSpeed(speed);
        break;
      default:
        // throw DriveModeHandlerError{};
        break;
    }
  }
  void GetBatteryPercent()
  {
    sjsu::LogInfo("%f%% of battery remaining on the drive",
                  state_of_charge_.StateOfCharge_MAX());
  }

 public:
  Wheels wheels_;
  MissionControlData mc_data_;

 private:
  Modes current_drive_mode_ = Modes::SpinMode;
  std::array<Wheel *, 3> wheel_array_{ wheels_.left_, wheels_.right_,
                                       wheels_.back_ };

  sjsu::common::StateOfCharge state_of_charge_;
  const int kExpectedArguments = 6;

  const float kZeroSpeed     = 0;
  const float kMaxTurnRadius = 45;
  const float kLerpStep      = 0.5;

  // TODO: Implement this logic once SOC is tested
  sjsu::common::StateOfCharge state_of_charge_;
};
}  // namespace sjsu::drive
