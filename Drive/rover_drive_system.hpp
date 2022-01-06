#pragma once

#include <stdio.h>

#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "utility/math/units.hpp"
#include "utility/math/map.hpp"

#include "../Common/state_of_charge.hpp"
#include "../Common/rover_system.hpp"
#include "../Common/esp.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
const char response_body_format[] =
    "\r\n\r\n{\n"
    "  \"heartbeat_count\": %d,\n"
    "  \"is_operational\": %d,\n"
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

  struct MissionControlData
  {
    int heartbeat_count = 0;
    int is_operational  = 0;
    char drive_mode     = 'S';
    int rotation_angle  = 0;
    int speed           = 0;
  };
  virtual ~RoverDriveSystem(){};
  RoverDriveSystem(Wheel & left_wheel, Wheel & right_wheel, Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  void Initialize()
  {
    sjsu::LogInfo("Initializing drive system...");
    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
    SetSpinMode();
  };

  /// Constructs parameters for an HTTP GET request
  /// @return ?heartbeat_count=0&is_operational=1&drive_mode=
  std::string GETParameters()
  {
    char request_parameter[300];
    snprintf(
        request_parameter, 300,
        "?heartbeat_count=%d&is_operational=%d&drive_mode=%c&battery=%d"
        "&left_wheel_speed=%d&left_wheel_angle=%d&right_wheel_speed=%d&right_"
        "wheel_angle=%d&back_wheel_speed=%d&back_wheel_angle=%d",
        heartbeat_count_, mc_data_.is_operational, current_mode_,
        state_of_charge_, left_wheel_.GetHubSpeed(),
        left_wheel_.GetSteerAngle(), right_wheel_.GetHubSpeed(),
        right_wheel_.GetSteerAngle(), back_wheel_.GetHubSpeed(),
        back_wheel_.GetSteerAngle());
    return request_parameter;
  };

  /// Parses the GET requests response and updates the mission control variables
  void ParseJSONResponse(std::string & response)
  {
    int actual_arguments =
        sscanf(response.c_str(), response_body_format,
               &mc_data_.heartbeat_count, &mc_data_.is_operational,
               &mc_data_.drive_mode, &mc_data_.speed, &mc_data_.rotation_angle);

    if (actual_arguments != kExpectedArguments)
    {
      sjsu::LogError("Arguments# %d != expected# %d!", actual_arguments,
                     kExpectedArguments);
      throw ParseError{};
    }
  };

  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation, L/R/B = Left/Right/Back Wheel
  void HandleRoverMovement()
  {
    if (!IsOperational() || !IsSynced())
    {
      SetWheelSpeed(kZeroSpeed);
      return;
    }
    if (IsNewMode())
    {
      SetMode();
      return;
    }

    double angle = mc_data_.rotation_angle;
    double speed = mc_data_.speed;

    if (!IsNewMode() && IsOperational() && IsSynced())
    {
      sjsu::LogInfo("Handling %c movement...", current_mode_);
      switch (current_mode_)
      {
        case 'D': HandleDriveMode(speed, angle); break;
        case 'S': HandleSpinMode(speed); break;
        case 'T': HandleTranslationMode(speed, angle); break;
        case 'L':
        case 'R':
        case 'B': HandleSingularWheelMode(speed, angle); break;
        default:
          sjsu::LogError("Unable to assign drive mode handler!");
          SetWheelSpeed(kZeroSpeed);
          break;
      }
      IncrementHeartbeat();
    }
    else
    {
      SetWheelSpeed(kZeroSpeed);
      // TODO: Throw error if this is reached!
    }
  };

  /// Homes the wheels to face directly outward in-line with slip ring
  void HomeWheels()
  {
    // TODO: Need to implement non-sequential homing procedure
    SetWheelSpeed(kZeroSpeed);
    left_wheel_.HomeWheel();
    right_wheel_.HomeWheel();
    back_wheel_.HomeWheel();
  };

  /// Sets all wheels to the speed provided
  void SetWheelSpeed(double target_speed)
  {
    double left_wheel_speed  = left_wheel_.GetHubSpeed();
    double right_wheel_speed = right_wheel_.GetHubSpeed();
    double back_wheel_speed  = back_wheel_.GetHubSpeed();

    left_wheel_speed  = std::lerp(left_wheel_speed, target_speed, kLerpStep);
    right_wheel_speed = std::lerp(right_wheel_speed, target_speed, kLerpStep);
    back_wheel_speed  = std::lerp(back_wheel_speed, target_speed, kLerpStep);

    left_wheel_.SetHubSpeed(left_wheel_speed);
    right_wheel_.SetHubSpeed(right_wheel_speed);
    back_wheel_.SetHubSpeed(back_wheel_speed);
  };

  /// Prints the mc data and all the current wheel data
  void PrintRoverData()
  {
    printf("HEARTBEAT:\t%d\n", mc_data_.heartbeat_count);
    printf("OPERATIONAL:\t%d\n", mc_data_.is_operational);
    printf("DRIVE MODE:\t%d\n", current_mode_);
    printf("MC SPEED:\t%d\n", mc_data_.speed);
    printf("MC ANGLE:\t%d\n", mc_data_.rotation_angle);
    printf("WHEEL     SPEED     ANGLE\n");
    printf("=========================\n");
    left_wheel_.Print();
    right_wheel_.Print();
    back_wheel_.Print();
    printf("=========================\n");
  };

 private:
  /// Checks whether the rover got a new drive mode command
  bool IsNewMode()
  {
    if (current_mode_ != mc_data_.drive_mode)
    {
      sjsu::LogWarning("Rover was assigned new drive mode!");
      return true;
    }
    return false;
  }

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
  bool IsSynced()
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

  void IncrementHeartbeat()
  {
    heartbeat_count_++;
  }

  /// Stops the rover and sets a new mode.
  void SetMode()
  {
    sjsu::LogWarning("Switching rover into %c mode...", mc_data_.drive_mode);
    SetWheelSpeed(kZeroSpeed);
    switch (mc_data_.drive_mode)
    {
      case 'D': SetDriveMode(); break;
      case 'S': SetSpinMode(); break;
      case 'T': SetTranslationMode(); break;
      case 'L':
      case 'R':
      case 'B': SetSingleWheelMode(); break;
      default: sjsu::LogError("Unable to set drive mode!");
    };
    IncrementHeartbeat();
  };

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing forward
  void SetDriveMode()
  {
    HomeWheels();
    // TODO: Find the angles close enough for an effective drive mode
    const int left_wheel_angle  = -45;
    const int right_wheel_angle = -135;
    const int back_wheel_angle  = 90;
    left_wheel_.SetSteerAngle(left_wheel_angle);
    right_wheel_.SetSteerAngle(right_wheel_angle);
    back_wheel_.SetSteerAngle(back_wheel_angle);
    current_mode_ = 'D';
  };

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    HomeWheels();
    // TODO: Find the angles close enough for an effective spin mode
    const double left_wheel_angle  = 90;
    const double right_wheel_angle = 90;
    const double back_wheel_angle  = 90;
    left_wheel_.SetSteerAngle(left_wheel_angle);
    right_wheel_.SetSteerAngle(right_wheel_angle);
    back_wheel_.SetSteerAngle(back_wheel_angle);
    current_mode_ = 'S';
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    HomeWheels();
    // TODO: Find the angles close enough for an effective translation mode
    const double left_wheel_angle  = 0;
    const double right_wheel_angle = 60;
    const double back_wheel_angle  = 110;
    left_wheel_.SetSteerAngle(left_wheel_angle);
    right_wheel_.SetSteerAngle(right_wheel_angle);
    back_wheel_.SetSteerAngle(back_wheel_angle);
    current_mode_ = 'T';
  };

  void SetSingleWheelMode()
  {
    SetWheelSpeed(kZeroSpeed);
    current_mode_ = mc_data_.drive_mode;
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  void HandleDriveMode(double speed, double inner_wheel_angle)
  {
    inner_wheel_angle =
        std::clamp(inner_wheel_angle, -kMaxTurnRadius, kMaxTurnRadius);

    double outter_wheel_angle(GetOutterWheelDriveAngle(inner_wheel_angle));
    double back_wheel_angle(GetBackWheelDriveAngle(inner_wheel_angle));

    if (inner_wheel_angle > 0)
    {
      right_wheel_.SetSteerAngle(inner_wheel_angle);
      left_wheel_.SetSteerAngle(outter_wheel_angle);
      back_wheel_.SetSteerAngle(back_wheel_angle);
    }
    else
    {
      right_wheel_.SetSteerAngle(outter_wheel_angle);
      left_wheel_.SetSteerAngle(inner_wheel_angle);
      back_wheel_.SetSteerAngle(back_wheel_angle);
    }
    // TODO: Need logic for controling wheel speed for each wheel
    SetWheelSpeed(speed);
  };

  /// Calculates outer wheel angle based off inner wheel angle
  double GetOutterWheelDriveAngle(double inner_wheel_angle)
  {
    double outter_wheel_angle = 0.392 + 0.744 * abs(int(inner_wheel_angle)) +
                                -0.0187 * pow(abs(int(inner_wheel_angle)), 2) +
                                1.84E-04 * pow(abs(int(inner_wheel_angle)), 3);
    return (inner_wheel_angle > 0) ? outter_wheel_angle : -outter_wheel_angle;
  }

  /// Calculates back wheel angle based off inner wheel angle
  double GetBackWheelDriveAngle(double inner_wheel_angle)
  {
    double back_wheel_angle = -0.378 + -1.79 * abs(int(inner_wheel_angle)) +
                              0.0366 * pow(abs(int(inner_wheel_angle)), 2) +
                              -3.24E-04 * pow(abs(int(inner_wheel_angle)), 3);
    return (inner_wheel_angle > 0) ? back_wheel_angle : -back_wheel_angle;
  }

  /// Adjusts only the speed since rover spins in place
  void HandleSpinMode(double speed)
  {
    SetWheelSpeed(speed);
  };

  /// Adjusts all the wheels by keeping them in parallel
  void HandleTranslationMode(double speed, double angle)
  {
    // TODO: Temporary placeholder till further testing - Incorrect logic
    left_wheel_.SetSteerAngle(angle);
    right_wheel_.SetSteerAngle(angle);
    back_wheel_.SetSteerAngle(angle);
    SetWheelSpeed(speed);
  };

  /// Adjusts the hub speed and steer angle of the specified wheel
  void HandleSingularWheelMode(double speed, double angle)
  {
    switch (current_mode_)
    {
      case 'L':
        left_wheel_.SetSteerAngle(angle);
        left_wheel_.SetHubSpeed(speed);
        break;
      case 'R':
        right_wheel_.SetSteerAngle(angle);
        right_wheel_.SetHubSpeed(speed);
        break;
      case 'B':
        back_wheel_.SetSteerAngle(angle);
        back_wheel_.SetHubSpeed(speed);
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        // TODO: Throw error if this is reached!
        break;
    }
  };

  int heartbeat_count_ = 0;
  int state_of_charge_ = 90;
  char current_mode_   = 'S';

  const int kExpectedArguments = 5;
  const double kZeroSpeed      = 0;
  const double kMaxTurnRadius  = 45;
  const double kLerpStep       = 0.5;

 public:
  MissionControlData mc_data_;
  Wheel & left_wheel_;
  Wheel & right_wheel_;
  Wheel & back_wheel_;
  // TODO: Implement this logic once SOC is tested
  // sjsu::common::StateOfCharge & battery_;
};
}  // namespace sjsu::drive
