#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"

namespace sjsu::drive
{
/// Wheel class manages steering & hub motors for the rover.
class Wheel
{
 public:
  Wheel(std::string name,
        sjsu::RmdX & hub_motor,
        sjsu::RmdX & steer_motor,
        sjsu::Gpio & homing_pin)
      : name_(name),
        hub_motor_(hub_motor),
        steer_motor_(steer_motor),
        homing_pin_(homing_pin){};

  void Initialize()
  {
    hub_motor_.Initialize();
    steer_motor_.Initialize();
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  /// Gets the speed of the hub motor.
  int GetSpeed()
  {
    return hub_speed_.to<int>();
  };

  /// Gets the angle/position of the steering motor.
  int GetPosition()
  {
    return homing_offset_angle_.to<int>();
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  /// @param hub_speed the new speed of the wheel
  void SetHubSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    try
    {
       sjsu::LogInfo("made it to sethubspeed()");
       
       auto clampedHubSpeed = std::clamp(hub_speed, kMaxNegSpeed, kMaxPosSpeed);
      //static cast to use clampedHubSpeed in lerp function
      long double new_speed = static_cast<long double>(clampedHubSpeed);
      //intialize the variable we will use to store the revolutions_per_minute_t that is returned on each call to std::lerp
      long double lerp_speed = 0;
      //intialize the variable we will use to store the current speed of the motor after each call to lerp
      long double current_speed = 0;

      //want to make sure we are supposed to increase or decrease the speed when setting the speed
      bool increaseSpeed = true;
      if(hub_motor_.RequestFeedbackFromMotor().GetFeedback().speed > hub_speed)
      {
        increaseSpeed = false;
      }

      //lerp must continue as long as input speed(hub_speed) does not equal the output of the lerp function(lerp_speed) 
       while(hub_speed != std::clamp(units::angular_velocity::revolutions_per_minute_t{lerp_speed}, kMaxNegSpeed, hub_speed))
       {
        auto next_same_time = Uptime() + 100ms;

        //current_speed is pulled from the motor feedback and casted to long double 
        current_speed = static_cast<long double>(hub_motor_.RequestFeedbackFromMotor().GetFeedback().speed);

        //lerp returns a midpoint between current_speed and the new_speed
        lerp_speed = std::lerp(current_speed, new_speed, static_cast<long double>(0.5));

        
        //Set the speed of the motor and convert long double lerp_speed to revolutions_per_minute_t object
        //use std::clamp to make sure we don't set the speed higher/lower than the input speed.
        //When using std::clamp check if the fucntion is increasing or decreasing the speed to limit the max/min speeds
        if(increaseSpeed)
        {
            hub_motor_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerp_speed}, kMaxNegSpeed, hub_speed));
        }
        else
        {
            hub_motor_.SetSpeed(std::clamp(units::angular_velocity::revolutions_per_minute_t{lerp_speed}, hub_speed, kMaxPosSpeed));
        }
        


        sjsu::LogInfo("Set hub speed to %f", lerp_speed);

        //make sure 100ms have passed before setting the next speed
        while(Uptime() < next_same_time)
        {
          continue;
        }

       }
       
    }
    catch (const std::exception & e)
    {
      sjsu::LogInfo("made it to sethubspeed() - something broke!");
      throw e;
    }
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  /// @param rotation_angle positive angle (turn right), negative angle (left)
  void SetSteeringAngle(units::angle::degree_t rotation_angle)
  {
    auto clampedRotationAngle =
        std::clamp(rotation_angle, kMaxNegRotation, kMaxPosRotation);
    units::angle::degree_t difference_angle =
        (homing_offset_angle_ + clampedRotationAngle);

    steer_motor_.SetAngle(difference_angle, kSteeringSpeed);
    homing_offset_angle_ += clampedRotationAngle;
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  /// The mark is indicated by the GPIO being set to low
  void HomeWheel()
  {
    // TODO: Needs to be cleaned up - early prototype
    sjsu::LogWarning("Homing %s wheel...", name_.c_str());
    bool home_level = sjsu::Gpio::kHigh;
    if (homing_pin_.Read() == home_level)
    {
      sjsu::LogInfo("Wheel %s already homed", name_.c_str());
      return;
    }

    steer_motor_.SetSpeed(10_rpm);
    while (homing_pin_.Read() != home_level)
    {
      sjsu::LogInfo("spinning");
      continue;
      // break;  // for testing purposes - comment out
    }
    steer_motor_.SetSpeed(0_rpm);
    sjsu::LogInfo("Wheel %s homed!", name_.c_str());
  };

  std::string name_;          // Wheel name (i.e. left, right, back)
  sjsu::RmdX & hub_motor_;    // Controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  // Controls wheel alignment/angle
  units::angle::degree_t homing_offset_angle_                  = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_ = 0_rpm;

  const units::angle::degree_t kMaxPosRotation = 360_deg;
  const units::angle::degree_t kMaxNegRotation = -360_deg;
  const units::angular_velocity::revolutions_per_minute_t kMaxPosSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNegSpeed =
      -100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kSteeringSpeed =
      20_rpm;
  sjsu::Gpio & homing_pin_;
};
}  // namespace sjsu::drive
