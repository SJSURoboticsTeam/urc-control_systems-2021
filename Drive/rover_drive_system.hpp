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
const char mc_response_body[] =
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
    int heartbeat_count;
    int is_operational;
    char drive_mode;
    int rotation_angle;
    int speed;
  };

  RoverDriveSystem(Wheel & left_wheel, Wheel & right_wheel, Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  /// Initializes wheels and sets rover to operational starting mode (spin)
  void Initialize()
  {
    sjsu::LogInfo("Initializing drive system...");
    mc_data.is_operational = 1;

    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
    SetSpinMode();
  };

  /// Constructs GET request parameter
  /// @return Endpoint & parameters i.e. /drive?ex=param
  std::string GETParameters()
  {
    char req_param[300];
    snprintf(
        req_param, 300,
        "?heartbeat_count=%d&is_operational=%d&drive_mode=%c&battery=%d"
        "&left_wheel_speed=%d&left_wheel_angle=%d&right_wheel_speed=%d&right_"
        "wheel_angle=%d&back_wheel_speed=%d&back_wheel_angle=%d",
        heartbeat_count_, mc_data.is_operational, current_mode_,
        state_of_charge_, left_wheel_.GetSpeed(), left_wheel_.GetPosition(),
        right_wheel_.GetSpeed(), right_wheel_.GetPosition(),
        back_wheel_.GetSpeed(), back_wheel_.GetPosition());
    std::string request_parameter = req_param;
    return request_parameter;
  };

  /// Parses GET response body and assigns it to rover variables
  /// @param response JSON response body
  void ParseJSONResponse(std::string & response)
  {
    int arguments =
        sscanf(response.c_str(), mc_response_body, &mc_data.heartbeat_count,
               &mc_data.is_operational, &mc_data.drive_mode, &mc_data.speed,
               &mc_data.rotation_angle);

    if (arguments != 5)
    {
      throw ParseError{};
    }
  };

  /// Verifies that mission control is sending fresh commands
  bool isSyncedWithMissionControl()
  {
    if (mc_data.heartbeat_count == heartbeat_count_)
    {
      heartbeat_count_++;
      return true;
    }
    else
    {
      sjsu::LogError("Heartbeat not in sync. Resetting");
      heartbeat_count_ = 0;
      return false;
    }
  }

  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation, L/R/B = Left/Right/Back Wheel
  void HandleRoverMovement()
  {
    int angle = mc_data.rotation_angle;
    int speed = mc_data.speed;

    if (!isSyncedWithMissionControl())
    {
      // TODO: Throw an error here instead of setting wheel speed to 0
      SetWheelSpeed(kZeroSpeed);
    }

    // If current mode is same as mc mode value and rover is operational
    else if (mc_data.is_operational && (current_mode_ == mc_data.drive_mode))
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
          SetWheelSpeed(kZeroSpeed);
          sjsu::LogError("Unable to assign drive mode handler!");
          break;
      }
    }
    else
    {
      // If current mode is not same as mc mode value
      sjsu::LogWarning("Switching rover into %c mode...", mc_data.drive_mode);
      SetMode();
    }
  };

  /// HomeWheels all the wheels so the motors know their actual position.
  /// @return true if successfully moves wheels into home position
  void HomeWheels()
  {
    // TODO: Need to implement non-sequential homing procedure
    SetWheelSpeed(kZeroSpeed);
    left_wheel_.HomeWheel();
    right_wheel_.HomeWheel();
    back_wheel_.HomeWheel();
  };

  /// Sets all wheels to the speed provided. Wheel class handles max/min speeds
  /// @param speed the new movement speed of the rover
  void SetWheelSpeed(int target_speed)
  {
    int left_wheel_speed  = left_wheel_.GetSpeed();
    int right_wheel_speed = right_wheel_.GetSpeed();
    int back_wheel_speed  = back_wheel_.GetSpeed();

    left_wheel_speed  = std::lerp(left_wheel_speed, target_speed, kLerpStep);
    right_wheel_speed = std::lerp(right_wheel_speed, target_speed, kLerpStep);
    back_wheel_speed  = std::lerp(back_wheel_speed, target_speed, kLerpStep);

    left_wheel_speed  = std::clamp(left_wheel_speed, kZeroSpeed, target_speed);
    right_wheel_speed = std::clamp(right_wheel_speed, kZeroSpeed, target_speed);
    back_wheel_speed  = std::clamp(back_wheel_speed, kZeroSpeed, target_speed);

    left_wheel_.SetHubSpeed(left_wheel_speed);
    right_wheel_.SetHubSpeed(right_wheel_speed);
    back_wheel_.SetHubSpeed(back_wheel_speed);
  };

  /// Prints the mission control data & prints the current speed and steer angle
  /// of each wheel on the rover
  void PrintRoverData()
  {
    printf(
        "HEARTBEAT:\t%d\nOPERATIONAL:\t%d\nDRIVE MODE:\t%c\nMC "
        "SPEED:\t%d\nMC ANGLE:\t%d\n\n",
        mc_data.heartbeat_count, mc_data.is_operational, current_mode_,
        mc_data.speed, mc_data.rotation_angle);
    printf("%-10s%-10s%-10s\n", "WHEEL", "SPEED", "ANGLE");
    printf("=========================\n");
    printf("%-10s%-10d%-10d\n", "Left", left_wheel_.GetSpeed(),
           left_wheel_.GetPosition());
    printf("%-10s%-10d%-10d\n", "Right", right_wheel_.GetSpeed(),
           right_wheel_.GetPosition());
    printf("%-10s%-10d%-10d\n", "Back", back_wheel_.GetSpeed(),
           back_wheel_.GetPosition());
    printf("=========================\n");
  };

 private:
  /// Stops the rover and sets a new mode.
  void SetMode()
  {
    SetWheelSpeed(kZeroSpeed);  // Stops rover
    switch (mc_data.drive_mode)
    {
      case 'D': SetDriveMode(); break;
      case 'S': SetSpinMode(); break;
      case 'T': SetTranslationMode(); break;
      case 'L':
      case 'R':
      case 'B': SetExperimentalMode(); break;
      default: sjsu::LogError("Unable to set drive mode!");
    };
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
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
    current_mode_ = 'D';
  };

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    HomeWheels();
    // TODO: Find the angles close enough for an effective spin mode
    const int left_wheel_angle  = 90;
    const int right_wheel_angle = 90;
    const int back_wheel_angle  = 90;
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
    current_mode_ = 'S';
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    HomeWheels();
    // TODO: Find the angles close enough for an effective translation mode
    const int left_wheel_angle  = 0;
    const int right_wheel_angle = 60;
    const int back_wheel_angle  = 110;
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
    current_mode_ = 'T';
  };

  /// Stops rover and updates current drive mode
  void SetExperimentalMode()
  {
    SetWheelSpeed(kZeroSpeed);
    current_mode_ = mc_data.drive_mode;
  }

  int GetOutterWheelDriveAngle(int angle)
  {
    double result = 0.392 + 0.744 * abs(angle) + -0.0187 * pow(abs(angle), 2) +
                    1.84E-04 * pow(abs(angle), 3);
    return (angle > 0) ? result : -result;
  }

  int GetBackWheelDriveAngle(int angle)
  {
    double result = -0.378 + -1.79 * abs(angle) + 0.0366 * pow(abs(angle), 2) +
                    -3.24E-04 * pow(abs(angle), 3);
    return (angle > 0) ? result : -result;
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode.
  void HandleDriveMode(int speed, int angle)
  {
    if (angle > kMaxTurnRadius)
    {
      sjsu::LogError("Rover Steering Angle too high");
      angle = kMaxTurnRadius;
    }
    else if (angle < -kMaxTurnRadius)
    {
      sjsu::LogError("Angle too low");
    }

    int inner_wheel_angle = angle;

    int outter_wheel_angle(GetOutterWheelDriveAngle(inner_wheel_angle));
    int back_wheel_angle(GetBackWheelDriveAngle(inner_wheel_angle));

    //*For testing angles*

    if (angle > 0)
    {
      right_wheel_.SetSteeringAngle(inner_wheel_angle);
      left_wheel_.SetSteeringAngle(outter_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
    }
    else
    {
      right_wheel_.SetSteeringAngle(outter_wheel_angle);
      left_wheel_.SetSteeringAngle(inner_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
    }
    // TODO: Temporary placeholder till further testing - Incorrect logic
    SetWheelSpeed(speed);
  };

  /// Handles spin mode. Adjusts only the speed (aka the spin direction)
  void HandleSpinMode(int speed)
  {
    SetWheelSpeed(speed);
  };

  /// Handles translation mode. Adjusts all the wheels, keeping them parallel
  void HandleTranslationMode(int speed, int angle)
  {
    // TODO: Temporary placeholder till further testing - Incorrect logic
    left_wheel_.SetSteeringAngle(angle);
    right_wheel_.SetSteeringAngle(angle);
    back_wheel_.SetSteeringAngle(angle);
    SetWheelSpeed(speed);
  };

  void HandleSingularWheelMode(int speed, int angle)
  {
    switch (current_mode_)
    {
      case 'L':
        left_wheel_.SetSteeringAngle(angle);
        left_wheel_.SetHubSpeed(speed);
        break;
      case 'R':
        right_wheel_.SetSteeringAngle(angle);
        right_wheel_.SetHubSpeed(speed);
        break;
      case 'B':
        back_wheel_.SetSteeringAngle(angle);
        back_wheel_.SetHubSpeed(speed);
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Should not be reached!");
        break;
    }
  };

  int heartbeat_count_     = 0;
  int state_of_charge_     = 90;
  char current_mode_       = 'S';
  const int kZeroSpeed     = 0;
  const int kMaxTurnRadius = 45;
  const double kLerpStep   = 0.5;

 public:
  MissionControlData mc_data;
  Wheel & left_wheel_;
  Wheel & right_wheel_;
  Wheel & back_wheel_;

  // TODO: Implement this logic once SOC is tested
  // sjsu::common::StateOfCharge & battery_;
};
}  // namespace sjsu::drive
