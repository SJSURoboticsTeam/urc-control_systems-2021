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
      mc_data.is_operational = 1;
      left_wheel_.Initialize();
      right_wheel_.Initialize();
      back_wheel_.Initialize();
      HomeWheels();
      sjsu::LogInfo("Drive system intialized...");
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
      units::angle::degree_t angle(mc_data.rotation_angle);
      units::angular_velocity::revolutions_per_minute_t speed(mc_data.speed);
      // If current mode is same as mc mode value and rover is operational
      if (mc_data.is_operational && (current_mode_ == mc_data.drive_mode))
      {
        sjsu::LogInfo("Handling %c movement...", current_mode_);
        switch (current_mode_)
        {
          case 'D': HandleDriveMode(speed, angle); break;
          case 'S': HandleSpinMode(speed); break;
          case 'T': HandleTranslationMode(speed, angle); break;
          default:
            SetWheelSpeed(kZeroSpeed);
            sjsu::LogError("Unable to assign drive mode handler!");
            break;
        }
      }
      else
      {
        // If current mode is not same as mc mode value
        sjsu::LogInfo("Switching rover into %c mode...", mc_data.drive_mode);
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
    //Implement linear interpolation (exponential moving average)
    try
    {
      sjsu::LogInfo("made it to SetWheelSpeed()");
       
       auto clampedHubSpeed = std::clamp(speed, kMaxNegSpeed, kMaxPosSpeed);
      //static cast to use clampedHubSpeed in lerp function
      long double new_speed = static_cast<long double>(clampedHubSpeed);
      //intialize the variable we will use to store the revolutions_per_minute_t that is returned on each call to std::lerp
      long double lerpSpeed_leftWheel = 0;
      long double lerpSpeed_rightWheel = 0;
      long double lerpSpeed_backWheel = 0;
      //intialize the variable we will use to store the current speed of the motor after each call to lerp
      long double currentSpeed_leftWheel = 0;
      long double currentSpeed_rightWheel = 0;
      long double currentSpeed_backWheel = 0;

      //want to make sure we are supposed to increase or decrease the speed when setting the speed
      bool increaseSpeed_leftWheel = true;
      bool increaseSpeed_rightWheel = true;
      bool increaseSpeed_backWheel = true;
      if(left_wheel_.RequestFeedbackFromMotor().GetFeedback().speed > speed)
      {
        increaseSpeed_leftWheel = false;
      }
      if(right_wheel_.RequestFeedbackFromMotor().GetFeedback().speed > speed)
      {
        increaseSpeed_rightWheel = false;
      }
      if(back_wheel_.RequestFeedbackFromMotor().GetFeedback().speed > speed)
      {
        increaseSpeed_backWheel = false;
      }
     
     //current_speed is pulled from the motor feedback and casted to long double 
      currentSpeed_leftWheel = static_cast<long double>(left_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);
      currentSpeed_rightWheel = static_cast<long double>(right_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);
      currentSpeed_backWheel = static_cast<long double>(back_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);   
        
     for(int i = 0; i < 6; i++)
     {
       //lerp returns a midpoint between current_speed and the new_speed
       lerpSpeed_leftWheel = std::lerp(currentSpeed_leftWheel, new_speed, static_cast<long double>(0.5));
       lerpSpeed_rightWheel = std::lerp(currentSpeed_rightWheel, new_speed, static_cast<long double>(0.5));
       lerpSpeed_backWheel = std::lerp(currentSpeed_backWheel, new_speed, static_cast<long double>(0.5));
      
       if(increaseSpeed_leftWheel)
       {
           left_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, kMaxNegSpeed, speed));
       }
       else
       {
           left_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, speed, kMaxPosSpeed));
       }
       if(increaseSpeed_rightWheel)
       {
           right_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, kMaxNegSpeed, speed));
       }
       else
       {
           right_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, speed, kMaxPosSpeed));
       }
       if(increaseSpeed_backWheel)
       {
           back_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, kMaxNegSpeed, speed));
       }
       else
       {
           back_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, speed, kMaxPosSpeed));
       }
      
       currentSpeed_leftWheel = lerpSpeed_leftWheel;
       currentSpeed_rightWheel = lerpSpeed_rightWheel;
       currentSpeed_backWheel = lerpSpeed_backWheel;
      
       sjsu::Delay(100ms);
     }
    

     
/*
      //lerp must continue as long as input speed(hub_speed) does not equal the output of the lerp function(lerp_speed) 
       while((speed != std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, kMaxNegSpeed, speed)) && (speed != std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, kMaxNegSpeed, speed)) && (speed != std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, kMaxNegSpeed, speed)))
       {
        auto next_same_time = Uptime() + 100ms;

        //current_speed is pulled from the motor feedback and casted to long double 
        currentSpeed_leftWheel = static_cast<long double>(left_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);
        currentSpeed_rightWheel = static_cast<long double>(right_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);
        currentSpeed_backWheel = static_cast<long double>(back_wheel_.RequestFeedbackFromMotor().GetFeedback().speed);
        
        //lerp returns a midpoint between current_speed and the new_speed
        lerpSpeed_leftWheel = std::lerp(currentSpeed_leftWheel, new_speed, static_cast<long double>(0.5));
        lerpSpeed_rightWheel = std::lerp(currentSpeed_rightWheel, new_speed, static_cast<long double>(0.5));
        lerpSpeed_backWheel = std::lerp(currentSpeed_backWheel, new_speed, static_cast<long double>(0.5));

        
        //Set the speed of the motor and convert long double lerp_speed to revolutions_per_minute_t object
        //use std::clamp to make sure we don't set the speed higher/lower than the input speed.
        //When using std::clamp check if the fucntion is increasing or decreasing the speed to limit the max/min speeds
        if(increaseSpeed_leftWheel)
        {
            left_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, kMaxNegSpeed, speed));
        }
        else
        {
            left_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_leftWheel}, speed, kMaxPosSpeed));
        }
        if(increaseSpeed_rightWheel)
        {
            right_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, kMaxNegSpeed, speed));
        }
        else
        {
            right_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_rightWheel}, speed, kMaxPosSpeed));
        }
        if(increaseSpeed_backWheel)
        {
            back_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, kMaxNegSpeed, speed));
        }
        else
        {
            back_wheel_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerpSpeed_backWheel}, speed, kMaxPosSpeed));
        }
        
        */


        sjsu::LogInfo("Set wheel speed to %f for all wheels", lerp_speed);

        //make sure 100ms have passed before setting the next speed
        while(Uptime() < next_same_time)
        {
          continue;
        }

       }//While
    }//Try
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting wheels speed!");
      throw e;
    }
  }; //SetWheelSpeed()

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
      const units::angle::degree_t left_wheel_angle  = 45_deg;
      const units::angle::degree_t right_wheel_angle = 45_deg;
      const units::angle::degree_t back_wheel_angle  = 45_deg;
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
      const units::angle::degree_t left_wheel_angle  = 45_deg;
      const units::angle::degree_t right_wheel_angle = -45_deg;
      const units::angle::degree_t back_wheel_angle  = -180_deg;
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

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    try
    {
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
