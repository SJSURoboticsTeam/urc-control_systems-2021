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
const char message_format[] =
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
  std::string GETParameters()
  {
    try
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
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error constructing GET request parameter!");
      throw e;
    }
  };

  /// Parses GET response body and assigns it to rover variables
  /// @param response JSON response body
  void ParseJSONResponse(std::string & response)
  {
    int arguments =
        sscanf(response.c_str(), message_format, &mc_data.heartbeat_count,
               &mc_data.is_operational, &mc_data.drive_mode, &mc_data.speed,
               &mc_data.rotation_angle);

    // TODO: Throw an error when arguments not equal to expected
  };

  bool isSyncedWithMissionControl()
  {
    if (mc_data.heartbeat_count == heartbeat_count_)
    {
      sjsu::LogInfo("In Sync");
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
  /// D = Drive, S = Spin, T = Translation
  void HandleRoverMovement()
  {
    try
    {
      units::angle::degree_t angle(static_cast<float>(mc_data.rotation_angle));
      units::angular_velocity::revolutions_per_minute_t speed(
          static_cast<float>(mc_data.speed));

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
      // TODO: Need to implement non-sequential homing procedure
      SetWheelSpeed(kZeroSpeed);
      left_wheel_.HomeWheel();
      right_wheel_.HomeWheel();
      back_wheel_.HomeWheel();
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
    //cast speed to long double to pass in to std::lerp
    long double goal_speed = static_cast<long double>(speed);

    //get current speed of motors
    long double leftWheel_previous_speed = static_cast<long double>(left_wheel_.GetSpeed());
    long double rightWheel_previous_speed = static_cast<long double>(right_wheel_.GetSpeed());
    long double backWheel_previous_speed = static_cast<long double>(back_wheel_.GetSpeed());
    

    //lerp returns a midpoint between current speed and goal speed 
    long double lerps = 0.5;
    auto lerpSpeed_leftWheel = std::lerp(leftWheel_previous_speed, goal_speed, lerps);
    auto lerpSpeed_rightWheel = std::lerp(rightWheel_previous_speed, goal_speed, lerps);
    auto lerpSpeed_backWheel = std::lerp(backWheel_previous_speed, goal_speed, lerps);

    //set lerped speed
    left_wheel_.SetHubSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, 0_rpm, speed));
    right_wheel_.SetHubSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, 0_rpm, speed));
    back_wheel_.SetHubSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, 0_rpm, speed));

    sjsu::LogInfo("SetHubSpeed to %f for all wheels", static_cast<long double>(right_wheel_.GetSpeed())  );
      
      
    
  }; //SetWheelSpeed()

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
  double GetLeftWheelDriveAngle(double angle)
  {
        double left_angle = static_cast<double>(0.392 + 0.744 * x_angle + -0.0187 * pow(x_angle, 2) +
        1.84E-04 * pow(x_angle, 3));
        return left_angle;
  }
  double GetBackWheelDriveAngle(double angle)
  {
        double back_angle = static_cast<double>(-0.378 + -1.79 * x_angle + 0.0366 * pow(x_angle, 2) +
        -3.24E-04 * pow(x_angle, 3));
        return back_angle;
  }

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode.
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    units::angle::degree_t right_wheel_angle =
        angle;  // Needs to be set by server
    double x_angle = static_cast<double>(right_wheel_angle);

    double left_angle = calculateLeftTurnAngle(x_angle);
    double back_angle = calculateBackTurnAngle(x_angle);

    units::angle::degree_t left_wheel_angle(left_angle);
    units::angle::degree_t back_wheel_angle(back_angle);

    //*For testing angles*
    sjsu::LogInfo("\n Right Wheel: %f\n LeftWheel: %f\n Back Wheel: %f\n",
                  x_angle, left_angle, back_angle);

    right_wheel_.SetSteeringAngle(right_wheel_angle);
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
    // TODO: Temporary placeholder till further testing - Incorrect logic
    SetWheelSpeed(speed);
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

  int heartbeat_count_                                               = 0;
  char current_mode_                                                 = 'S';
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;

 public:
  MissionControlData mc_data;
  Wheel & left_wheel_;
  Wheel & right_wheel_;
  Wheel & back_wheel_;

  sjsu::common::StateOfCharge * state_of_charge_ =
      new sjsu::common::StateOfCharge();
  int state_of_charge_MAX_ =
      static_cast<int>(state_of_charge_->StateOfCharge_MAX());
  int state_of_charge_LTC_ =
      static_cast<int>(state_of_charge_->StateOfCharge_LTC());
};
}  // namespace sjsu::drive
