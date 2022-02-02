#pragma once

#include "utility/log.hpp"

#include "../Common/state_of_charge.hpp"
#include "../Common/rover_system.hpp"
#include "../Common/heartbeat.hpp"
#include "../Common/esp.hpp"
#include "wheel.hpp"
#include <array>

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
class RoverDriveSystem : public sjsu::common::RoverSystem
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

  struct MissionControlData : public RoverMissionControlData
  {
    int wheel_shift    = 0;
    char drive_mode    = 'S';
    int rotation_angle = 0;
    int speed          = 0;
  };

  RoverDriveSystem(Wheel & left_wheel, Wheel & right_wheel, Wheel & back_wheel)
  {
    wheel.left_ = &left_wheel;
    wheel.right_ = &right_wheel;
    wheel.back_ = &back_wheel;
  }

  virtual void Initialize() override
  {
    sjsu::LogInfo("Initializing drive system...");
    wheel.left_->Initialize();
    wheel.right_->Initialize();
    wheel.back_->Initialize();
    

    SetSpinMode();
    sjsu::LogInfo("Drive system initialized!");
  }

  // example Call: drive.switchLegOrientation(drive, back_wheel, right_wheel, back_wheel)
  void SwitchLegOrientation(int position)
  {
    /*
    [0] = left right back
    [1] = back left right
    [2] = right back left
    */
    if(IsStopped())
    {
        wheel.left_= wheels[(position + 0) % 3];
        wheel.right_ = wheels[(position + 1) % 3];
        wheel.back_ = wheels[(position + 2) % 3];
    }
  };

  /// Constructs parameters for an HTTP GET request
  /// @return ?heartbeat_count=0&is_operational=1&drive_mode=S ...
  std::string GETParameters() override
  {
    char request_parameter[300];
    snprintf(
        request_parameter, 300,
        "?heartbeat_count=%d&is_operational=%d&wheel_shift=%d&drive_mode=%c&battery=%d"
        "&left_wheel_speed=%d&left_wheel_angle=%d&right_wheel_speed=%d&right_"
        "wheel_angle=%d&back_wheel_speed=%d&back_wheel_angle=%d",
        GetHeartbeatCount(), mc_data_.is_operational, mc_data_.wheel_shift, current_mode_,
        state_of_charge_, wheel.left_->GetHubSpeed(),
        wheel.left_->GetSteerAngle(), wheel.right_->GetHubSpeed(),
        wheel.right_->GetSteerAngle(), wheel.back_->GetHubSpeed(),
        wheel.back_->GetSteerAngle());
    return request_parameter;
  }

  /// Parses the GET requests response and updates the mission control variables
  void ParseJSONResponse(std::string & response) override
  {
    int actual_arguments =
        sscanf(response.c_str(), response_body_format,
               &mc_data_.heartbeat_count, &mc_data_.is_operational, &mc_data_.wheel_shift,
               &mc_data_.drive_mode, &mc_data_.speed, &mc_data_.rotation_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  }


  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation, L/R/B = Left/Right/Back Wheel
  void HandleRoverMovement() override
  {
    if (!IsHeartbeatSynced(mc_data_.heartbeat_count))
    {
      SetWheelSpeed(kZeroSpeed);
      return;
    }
    if (!IsOperational())
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

    switch (current_mode_)
    {
      case 'D': HandleDriveMode(speed, angle); break;
      case 'S': HandleSpinMode(speed); break;
      case 'T': HandleTranslationMode(speed, angle); break;
      case 'L':
      case 'R':
      case 'B': HandleSingularWheelMode(speed, angle); break;
      default:
        StopWheels();
        // throw DriveModeHandlerError{};
        break;
    }
  }

  /// Checks if the rover is operational
  bool IsOperational()
  {
    if (mc_data_.is_operational != 1)
    {
      sjsu::LogWarning("Drive mode is not operational!");
      return false;
    }
    return true;
  }

  /// Checks if the rover got a new drive mode command
  bool IsNewMode()
  {
    if (current_mode_ != mc_data_.drive_mode)
    {
      sjsu::LogWarning("Rover was assigned new drive mode!");
      return true;
    }
    return false;
  }

  char GetCurrentMode()
  {
    return current_mode_;
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
    return (wheel.left_->GetHubSpeed() == 0 && wheel.right_->GetHubSpeed() == 0 &&
            wheel.back_->GetHubSpeed() == 0);
  }

  /// Slowly lerps the wheels toward the target speed
  void SetWheelSpeed(float target_speed)
  {
    float left_wheel_speed  = float(wheel.left_->GetHubSpeed());
    float right_wheel_speed = float(wheel.right_->GetHubSpeed());
    float back_wheel_speed  = float(wheel.back_->GetHubSpeed());

    left_wheel_speed  = std::lerp(left_wheel_speed, target_speed, kLerpStep);
    right_wheel_speed = std::lerp(right_wheel_speed, target_speed, kLerpStep);
    back_wheel_speed  = std::lerp(back_wheel_speed, target_speed, kLerpStep);

    wheel.left_->SetHubSpeed(left_wheel_speed);
    wheel.right_->SetHubSpeed(right_wheel_speed);
    wheel.back_->SetHubSpeed(back_wheel_speed);
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
      if (!wheel.left_->IsHomed())
      {
        wheel.left_->SetSteerAngle(0);
      }
      if (!wheel.right_->IsHomed())
      {
        wheel.right_->SetSteerAngle(0);
      }
      if (!wheel.back_->IsHomed())
      {
        wheel.back_->SetSteerAngle(0);
      }
      sjsu::Delay(50ms);
    }
    sjsu::LogInfo("Wheels homed!");
  }

  bool AllWheelsAreHomed()
  {
    return (wheel.left_->IsHomed() && wheel.right_->IsHomed() &&
            wheel.back_->IsHomed());
  }

  /// Prints the mc data and all the current wheel data
  void PrintRoverData() override
  {
    printf("HEARTBEAT:\t%d\n", mc_data_.heartbeat_count);
    printf("OPERATIONAL:\t%d\n", mc_data_.is_operational);
    printf("WHEEL SHIFT:\t%d\n", mc_data_.wheel_shift);
    printf("DRIVE MODE:\t%d\n", current_mode_);
    printf("MC SPEED:\t%d\n", mc_data_.speed);
    printf("MC ANGLE:\t%d\n", mc_data_.rotation_angle);
    printf("WHEEL     SPEED     ANGLE     ENCODER-POS\n");
    printf("=========================================\n");
    wheel.left_->Print();
    wheel.right_->Print();
    wheel.back_->Print();
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
      case 'D': SetDriveMode(); break;
      case 'S': SetSpinMode(); break;
      case 'T': SetTranslationMode(); break;
      case 'L':
      case 'R':
      case 'B': SetSingleWheelMode(); break;
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
    const int left_wheel_angle  = -45;
    const int right_wheel_angle = -135;
    const int back_wheel_angle  = 90;
    wheel.left_->SetSteerAngle(left_wheel_angle);
    wheel.right_->SetSteerAngle(right_wheel_angle);
    wheel.back_->SetSteerAngle(back_wheel_angle);
    current_mode_ = 'D';
  }

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    const float left_wheel_angle  = 90;
    const float right_wheel_angle = 90;
    const float back_wheel_angle  = 90;
    wheel.left_->SetSteerAngle(left_wheel_angle);
    wheel.right_->SetSteerAngle(right_wheel_angle);
    wheel.back_->SetSteerAngle(back_wheel_angle);
    current_mode_ = 'S';
  }

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    // TODO: Find the angles for translation mode
    const float left_wheel_angle  = 0;
    const float right_wheel_angle = 60;
    const float back_wheel_angle  = 110;
    wheel.left_->SetSteerAngle(left_wheel_angle);
    wheel.right_->SetSteerAngle(right_wheel_angle);
    wheel.back_->SetSteerAngle(back_wheel_angle);
    current_mode_ = 'T';
  }

  void SetSingleWheelMode()
  {
    current_mode_ = mc_data_.drive_mode;
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  void HandleDriveMode(float speed, float inner_wheel_angle)
  {
    inner_wheel_angle =
        std::clamp(inner_wheel_angle, -kMaxTurnRadius, kMaxTurnRadius);

    float outter_wheel_angle(GetOutterWheelDriveAngle(inner_wheel_angle));
    float back_wheel_angle(GetBackWheelDriveAngle(inner_wheel_angle));

    if (inner_wheel_angle > 0)
    {
      wheel.right_->SetSteerAngle(inner_wheel_angle);
      wheel.left_->SetSteerAngle(outter_wheel_angle);
      wheel.back_->SetSteerAngle(back_wheel_angle);
    }
    else
    {
      wheel.right_->SetSteerAngle(outter_wheel_angle);
      wheel.left_->SetSteerAngle(inner_wheel_angle);
      wheel.back_->SetSteerAngle(back_wheel_angle);
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
    wheel.left_->SetSteerAngle(angle);
    wheel.right_->SetSteerAngle(angle);
    wheel.back_->SetSteerAngle(angle);
    SetWheelSpeed(speed);
  }

  /// Adjusts the hub speed and steer angle of the specified wheel
  void HandleSingularWheelMode(float speed, float angle)
  {
    switch (current_mode_)
    {
      case 'L':
        wheel.left_->SetSteerAngle(angle);
        wheel.left_->SetHubSpeed(speed);
        break;
      case 'R':
        wheel.right_->SetSteerAngle(angle);
        wheel.right_->SetHubSpeed(speed);
        break;
      case 'B':
        wheel.back_->SetSteerAngle(angle);
        wheel.back_->SetHubSpeed(speed);
        break;
    }
  }

  int state_of_charge_ = 90;
  char current_mode_   = 'S';

  const int kExpectedArguments = 6;
  const float kZeroSpeed       = 0;
  const float kMaxTurnRadius   = 45;
  const float kLerpStep        = 0.5;


 public:
  MissionControlData mc_data_;
  std::array<Wheel*, 3> wheels{wheel.left_, wheel.right_, wheel.back_};
  struct wheels_{
    Wheel * left_;
    Wheel * right_;
    Wheel * back_;
  }wheel;

 public:
  wheels_ getWheels() {
    return wheel;
  }
  // TODO: Implement this logic once SOC is tested
  // sjsu::common::StateOfCharge & battery_;
};
}  // namespace sjsu::drive
