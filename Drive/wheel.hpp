#pragma once

#include "devices/actuators/servo/rmd_x.hpp"
#include "peripherals/lpc40xx/gpio.hpp"
#include "devices/switches/button.hpp"

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
    // homing_pin_.GetPin().settings.Floating();  // for real
    homing_pin_.GetPin().settings.PullDown();  // for testing
    homing_pin_.Initialize();
    homing_pin_.SetAsInput();
  };

  /// Gets the speed of the hub motor.
  int GetSpeed()
  {
    // TODO: Grab actual speed from RMD GetFeedback()
    return hub_speed_.to<int>();
  };

  /// Gets the angle/position of the steering motor.
  int GetPosition()
  {
    // TODO: Grab actual angle from RMD GetFeedback()
    return homing_offset_angle_.to<int>();
  };

  /// Sets the speed of the hub motor. Will not surpass max/min value
  /// @param hub_speed the new speed of the wheel
  void SetHubSpeed(units::angular_velocity::revolutions_per_minute_t hub_speed)
  {
    hub_motor_.SetSpeed(hub_speed);
    // hub_speed_ = hub_speed;
    // auto clampedHubSpeed = std::clamp(hub_speed, kMaxNegSpeed, kMaxPosSpeed);
    // hub_motor_.SetSpeed(clampedHubSpeed);
    // hub_speed_ = clampedHubSpeed;
  }

  /// Adjusts the steer motor by the provided rotation angle/degree.
  /// @param rotation_angle positive angle (turn right), negative angle (left)
  void SetSteeringAngle(units::angle::degree_t rotation_angle)
  {
    // auto clampedRotationAngle =
    //     std::clamp(rotation_angle, kMaxNegRotation, kMaxPosRotation);
    // units::angle::degree_t difference_angle =
    //     (homing_offset_angle_ + clampedRotationAngle);

    steer_motor_.SetAngle(rotation_angle, kSteerSpeed);
    // homing_offset_angle_ = rotation_angle;
  };

  /// Sets the wheel back in its homing position by finding mark in slip ring.
  /// The mark is indicated by the GPIO being set to low
  void HomeWheel()
  {
    // For testing homing procedure
    sjsu::Button homing_button(homing_pin_);
    homing_button.Initialize();

    sjsu::LogWarning("Homing %s wheel...", name_.c_str());
    bool home_level = sjsu::Gpio::kHigh;

    // Loop thru all possible angles (0-360)
    // Increments through every angle
    // When the homing pin is high stop incrementing and update homing offset
    SetSteeringAngle(0_deg);
    sjsu::Delay(5s);

    for (int i = 0; i < (360); i += 2)
    {
      sjsu::LogInfo("%d", i);
      SetSteeringAngle(units::angle::degree_t{ i });
      sjsu::Delay(50ms);
      if (homing_pin_.Read() == home_level)
      {
        homing_offset_angle_ = units::angle::degree_t{ i };
        break;
      }
    }
    // Old Method VV
    // steer_motor_.SetSpeed(kSteerSpeed);
    // while (homing_pin_.Read() != home_level || homing_button.Pressed())
    // {
    // }
    // steer_motor_.SetSpeed(kZeroSpeed);
    sjsu::LogInfo("Homing %s wheel is done!", name_.c_str());
  };

  std::string name_;          // Wheel name (i.e. left, right, back)
  sjsu::RmdX & hub_motor_;    // Controls tire direction (fwd/rev) & speed
  sjsu::RmdX & steer_motor_;  // Controls wheel alignment/angle
  units::angle::degree_t homing_offset_angle_                         = 0_deg;
  units::angular_velocity::revolutions_per_minute_t hub_speed_        = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed  = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kSteerSpeed = 20_rpm;

  const units::angle::degree_t kMaxPosRotation = 360_deg;
  const units::angle::degree_t kMaxNegRotation = -360_deg;

  const units::angular_velocity::revolutions_per_minute_t kMaxPosSpeed =
      100_rpm;
  const units::angular_velocity::revolutions_per_minute_t kMaxNegSpeed =
      -100_rpm;
  sjsu::Gpio & homing_pin_;
};
}  // namespace sjsu::drive
