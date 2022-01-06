#pragma once

#include <stdio.h>

#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "utility/math/units.hpp"
#include "utility/math/map.hpp"

#include "../Common/esp.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
class RoverDriveSystem
{
 public:
  struct MissionControlData
  {
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
    try
    {
      sjsu::LogInfo("Initializing drive system...");
      mc_data.is_operational = 1;
      left_wheel_.Initialize();
      right_wheel_.Initialize();
      back_wheel_.Initialize();
      HomeWheels();
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error initializing!");
      throw e;
    }
  };

  /// Constructs GET request parameter
  /// @return Endpoint & parameters i.e. /drive?ex=param
  std::string GETRequestParameters()
  {
    try
    {
      char reqParam[250];
      snprintf(reqParam, 300,
               "drive?is_operational=%d&drive_mode=%c&battery=%d&left_wheel_"
               "speed=%d&left_wheel_angle=%d&right_wheel_speed=%d&right_"
               "wheel_angle=%d&back_wheel_speed=%d&back_wheel_angle=%d",
               mc_data.is_operational, current_mode_, state_of_charge_,
               left_wheel_.GetSpeed(), left_wheel_.GetPosition(),
               right_wheel_.GetSpeed(), right_wheel_.GetPosition(),
               back_wheel_.GetSpeed(), back_wheel_.GetPosition());
      std::string requestParameter = reqParam;
      return requestParameter;
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error constructing GET request parameter!");
      throw e;
    }
  };

  /// Parses GET response body and assigns it to rover variables
  /// @param response JSON response body
  void ParseJSONResponse(std::string response)
  {
    try
    {
      // TODO: Account for heartbeat procedure i.e. "messageCount": 132
      sscanf(
          response.c_str(),
          R"({ "is_operational": %d, "drive_mode": "%c", "speed": %d, "angle": %d })",
          &mc_data.is_operational, &mc_data.drive_mode, &mc_data.speed,
          &mc_data.rotation_angle);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error parsing GET response!");
      throw e;
    }
  };

  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation
  void HandleRoverMovement()
  {
    try
    {
      units::angle::degree_t angle(static_cast<float>(mc_data.rotation_angle));
      units::angular_velocity::revolutions_per_minute_t speed(
          static_cast<float>(mc_data.speed));
      // If current mode is same as mc mode value and rover is operational
      if (mc_data.is_operational && (current_mode_ == mc_data.drive_mode))
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
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling movement!");
      throw e;
    }
  };

  /// HomeWheels all the wheels so the motors know their actual position.
  /// @return true if successfully moves wheels into home position
  void HomeWheels()
  {
    try
    {
      // Simultaneous wheel homing
      SetWheelSpeed(kZeroSpeed);
      sjsu::Delay(50ms);
      for (units::angle::degree_t angle = 0_deg; angle < 360_deg; angle += 2_deg)
      {
        if(!left_wheel_.IsWheelHomed)
        {
          left_wheel_.SetSteeringAngle(angle);
        }
        if(!right_wheel_.IsWheelHomed)
        {
          right_wheel_.SetSteeringAngle(angle);
        }
        if(!back_wheel_.IsWheelHomed)
        {
          back_wheel_.SetSteeringAngle(angle);
        }
      }
      //
      sjsu::Delay(50ms);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error homing the wheels!");
      throw e;
    }
  };

  /// Sets all wheels to the speed provided. Wheel class handles max/min speeds
  /// @param speed the new movement speed of the rover
  void SetWheelSpeed(units::angular_velocity::revolutions_per_minute_t speed)
  {
    // TODO: Implement linear interpolation (exponential moving average)
    try
    {
      left_wheel_.SetHubSpeed(speed);
      right_wheel_.SetHubSpeed(speed);
      back_wheel_.SetHubSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting wheels speed!");
      throw e;
    }
  };

  /// Prints the mission control data & prints the current speed and steer angle
  /// of each wheel on the rover
  void PrintRoverData()
  {
    printf(
        "OPERATIONAL:\t%d\nDRIVE MODE:\t%c\nMC SPEED:\t%d\nMC ANGLE:\t%d\n\n",
        mc_data.is_operational, current_mode_, mc_data.speed,
        mc_data.rotation_angle);
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
    try
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
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting drive mode!");
      throw e;
    }
  };

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing forward
  void SetDriveMode()
  {
    try
    {
      HomeWheels();
      // TODO: Find the angles close enough for an effective drive mode
      const units::angle::degree_t left_wheel_angle  = -45_deg;
      const units::angle::degree_t right_wheel_angle = -135_deg;
      const units::angle::degree_t back_wheel_angle  = 90_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'D';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting drive mode!");
      throw e;
    }
  };

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    try
    {
      HomeWheels();
      // TODO: Find the angles close enough for an effective spin mode
      const units::angle::degree_t left_wheel_angle  = 90_deg;
      const units::angle::degree_t right_wheel_angle = 90_deg;
      const units::angle::degree_t back_wheel_angle  = 90_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'S';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting spin mode!");
      throw e;
    }
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    try
    {
      HomeWheels();
      // TODO: Find the angles close enough for an effective translation mode
      const units::angle::degree_t left_wheel_angle  = 0_deg;
      const units::angle::degree_t right_wheel_angle = 60_deg;
      const units::angle::degree_t back_wheel_angle  = 110_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'T';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting translation mode!");
      throw e;
    }
  };

  /// Stops rover and updates current drive mode
  void SetExperimentalMode()
  {
    try
    {
      SetWheelSpeed(kZeroSpeed);
      current_mode_ = mc_data.drive_mode;
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting translation mode!");
      throw e;
    }
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    try
    {
      // TODO: Temporary placeholder till further testing - Incorrect logic
      back_wheel_.SetSteeringAngle(angle);
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling drive mode!");
      throw e;
    }
  };

  /// Handles spin mode. Adjusts only the speed (aka the spin direction)
  void HandleSpinMode(units::angular_velocity::revolutions_per_minute_t speed)
  {
    try
    {
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling spin mode!");
      throw e;
    }
  };

  /// Handles translation mode. Adjusts all the wheels, keeping them parallel
  void HandleTranslationMode(
      units::angular_velocity::revolutions_per_minute_t speed,
      units::angle::degree_t angle)
  {
    try
    {
      // TODO: Temporary placeholder till further testing - Incorrect logic
      left_wheel_.SetSteeringAngle(angle);
      right_wheel_.SetSteeringAngle(angle);
      back_wheel_.SetSteeringAngle(angle);
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling translation mode!");
      throw e;
    }
  };

  void HandleSingularWheelMode(
      units::angular_velocity::revolutions_per_minute_t speed,
      units::angle::degree_t angle)
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

  char current_mode_   = 'S';
  int state_of_charge_ = 90;  // TODO - hardcoded for now

  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;

 public:
  MissionControlData mc_data;
  Wheel & left_wheel_;
  Wheel & right_wheel_;
  Wheel & back_wheel_;
};
}  // namespace sjsu::drive