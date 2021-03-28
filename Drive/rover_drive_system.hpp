#pragma once

#include <stdio.h>

#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "utility/math/units.hpp"
#include "utility/math/map.hpp"

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

  /// Mission controls possible input values
  struct MissionControlData
  {
    int is_operational;
    char drive_mode;
    float rotation_angle;
    float speed;
    char response_body[300];
    char request_parameter[300];
  };

  RoverDriveSystem(sjsu::drive::Wheel & left_wheel,
                   sjsu::drive::Wheel & right_wheel,
                   sjsu::drive::Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  void Initialize()
  {
    mission_control_data_.is_operational = true;
    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
    SetMode();
  };

  // Handles data transfer for rover drive system to mission control
  /// @return returns true if connection is good and request is successful
  bool ExchangeMissionControlData()
  {
    if (GETRequest() && ParseGETResponseBody())
    {
      Move();
      return true;
    }
    else
    {
      SetMode();
      sjsu::LogError("Bad mission control response - stopping rover...");
      return false;
    }
  }

  /// Main function for handling all the rover drive system functionality.
  /// drive_mode updates drive mode if new mode is different.
  /// rotation_angle adjusts wheel position depending on mode.
  /// speed adjusts the movement speed of the rover.
  void Move()
  {
    if (mission_control_data_.is_operational)
    {
      if (mission_control_data_.drive_mode != static_cast<char>(current_mode_))
      {
        SetMode(mission_control_data_.drive_mode);
      }
      else
      {
        HandleRoverMovement(mission_control_data_.rotation_angle,
                            mission_control_data_.speed);
      }
    }
    else
    {
      sjsu::LogError("Rover is_operational is set to false!");
    }
  };

  /// Resets all the wheels so the motors know their actual position.
  /// @return true if successfully resets wheels into start position
  bool Reset()
  {
    SetMode();
    return true;
  };

 private:
  /// Sets all wheels to the speed provided. Wheel class handles max/min speeds
  /// @param speed the new movement speed of the rover
  void SetWheelSpeed(units::angular_velocity::revolutions_per_minute_t speed)
  {
    // TODO: Implement linear interploation (exponentional moving average) to
    // smooth out changes in speed.
    left_wheel_.SetHubSpeed(speed);
    right_wheel_.SetHubSpeed(speed);
    back_wheel_.SetHubSpeed(speed);
  };

  /// Send HTTP GET request and updates /drive/status endpoint with rover's
  /// telemetry readings
  /// @return true if GET request is 200 or < 300 ?
  bool GETRequest()
  {
    GETRequestParameterConstructor();
    // TODO: Verify GET param is correct
    // TODO: Need to implement Esp class & construct params for GET request
    // EX: &mission_control_data_.response_body =
    // esp_.GETDrive(mission_control_data_.request_parameter);
    bool successful_request = true;
    return successful_request;
  };

  /// Constructs GET request parameter
  void GETRequestParameterConstructor()
  {
    int parameter_size = snprintf(
        mission_control_data_.request_parameter, 300,
        "?is_operational=%d&drive_mode=%c&battery=%d&left_wheel_speed=%f&"
        "left_wheel_angle=%f&right_wheel_speed=%f&right_wheel_angle=%f&"
        "back_wheel_speed=%f&back_wheel_angle=%f",
        mission_control_data_.is_operational, static_cast<char>(current_mode_),
        state_of_charge_, left_wheel_.GetSpeed(), left_wheel_.GetPosition(),
        right_wheel_.GetSpeed(), right_wheel_.GetPosition(),
        back_wheel_.GetSpeed(), back_wheel_.GetPosition());
  };

  /// Prints the speed and position/angle of each wheel on the rover
  void PrintRoverData()
  {
    sjsu::LogInfo("is_operational: %d", mission_control_data_.is_operational);
    sjsu::LogInfo("drive_mode: %c", static_cast<char>(current_mode_));
    sjsu::LogInfo("state of charge: %d", state_of_charge_);
    sjsu::LogInfo("left wheel speed: %f", left_wheel_.GetSpeed());
    sjsu::LogInfo("left wheel position: %f", left_wheel_.GetPosition());
    sjsu::LogInfo("right wheel speed: %f", right_wheel_.GetSpeed());
    sjsu::LogInfo("right wheel position: %f", right_wheel_.GetPosition());
    sjsu::LogInfo("back wheel speed: %f", back_wheel_.GetSpeed());
    sjsu::LogInfo("back wheel position: %f", back_wheel_.GetPosition());
  };

  /// Parses incoming JSON data from mission control to rover
  /// @return true if GET response successfully parsed with correct # of cmds
  /// and valid drive mode entered
  bool ParseGETResponseBody()
  {
    const int expected_num_cmds = 4;
    int parsed_num_cmds         = sscanf(
        mission_control_data_.response_body,
        R"({"is_opertaional": %d, "drive_mode": "%c", "speed": %f, "angle": %f})",
        &mission_control_data_.is_operational,
        &mission_control_data_.drive_mode, &mission_control_data_.speed,
        &mission_control_data_.rotation_angle);
    if ((parsed_num_cmds == expected_num_cmds) &&
        (mission_control_data_.drive_mode == 'D' ||
         mission_control_data_.drive_mode == 'S' ||
         mission_control_data_.drive_mode == 'T'))
    {
      return true;
    }
    else
    {
      sjsu::LogError("Error parsing response from Mission Control!");
      return false;
    }
  };

  /// Sets the new driving mode for the rover. Rover will stop before switching
  /// @param mode Three Modes: D (drive), S (spin), T (translation)
  void SetMode(char mode = 'S')
  {
    switch (mode)
    {
      case 'D':
        current_mode_ = Mode::kDrive;
        SetWheelSpeed(kZeroSpeed);
        SetDriveMode();
        sjsu::LogInfo("Drive mode set");
        break;
      case 'S':
        current_mode_ = Mode::kSpin;
        SetWheelSpeed(kZeroSpeed);
        SetSpinMode();
        sjsu::LogInfo("Spin mode set");
        break;
      case 'T':
        current_mode_ = Mode::kTranslation;
        SetWheelSpeed(kZeroSpeed);
        SetTranslationMode();
        sjsu::LogInfo("Translation mode set");
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Unable to assign drive mode!");
    };
  };

  /// Handles the rover movement depending on the mode
  /// @param rotation_angle adjusts wheel position depending on mode.
  /// @param wheel_speed adjusts the movement speed of the rover.
  void HandleRoverMovement(float roatation_angle, float wheel_speed)
  {
    units::angle::degree_t angle(roatation_angle);
    units::angular_velocity::revolutions_per_minute_t speed(wheel_speed);

    switch (current_mode_)
    {
      case Mode::kDrive:
        sjsu::LogInfo("Driving...");
        HandleDriveMode(speed, angle);
        break;
      case Mode::kSpin:
        sjsu::LogInfo("Spining...");
        HandleSpinMode(speed);
        break;
      case Mode::kTranslation:
        sjsu::LogInfo("Translating...");
        HandleTranslationMode(speed, angle);
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Unable to assign drive mode handler!");
        break;
    }
    current_speed_ = speed;
  };

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing foward
  void SetDriveMode()
  {
    const units::angle::degree_t left_wheel_angle  = -45_deg;
    const units::angle::degree_t right_wheel_angle = -135_deg;
    const units::angle::degree_t back_wheel_angle  = 90_deg;
    SetSpinMode();
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
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
    const units::angle::degree_t left_wheel_angle  = 45_deg;
    const units::angle::degree_t right_wheel_angle = -45_deg;
    const units::angle::degree_t back_wheel_angle  = -180_deg;
    SetSpinMode();
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
  };

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    back_wheel_.SetSteeringAngle(angle);
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
    left_wheel_.SetSteeringAngle(angle);
    right_wheel_.SetSteeringAngle(angle);
    back_wheel_.SetSteeringAngle(angle);
    SetWheelSpeed(speed);
  };

  int state_of_charge_ = 57;  // TODO: Implement at some point
  sjsu::drive::Wheel & left_wheel_;
  sjsu::drive::Wheel & right_wheel_;
  sjsu::drive::Wheel & back_wheel_;
  MissionControlData mission_control_data_;
  sjsu::drive::RoverDriveSystem::Mode current_mode_ = Mode::kSpin;
  units::angular_velocity::revolutions_per_minute_t current_speed_   = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;
};
}  // namespace sjsu::drive
