#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
// the Joint class is used for the rotunda, elbow, and shoulder motors.
class Joint
{
 public:
  struct Acceleration
  {
    float x = 0;
    float y = 0;
    float z = 0;
  };
  Joint(sjsu::RmdX & joint_motor, sjsu::Mpu6050 & accelerometer)
      : motor(joint_motor), mpu(accelerometer)
  {
  }

  Joint(sjsu::RmdX & joint_motor,
        sjsu::Mpu6050 & accelerometer,
        units::angle::degree_t min_angle,
        units::angle::degree_t max_angle,
        units::angle::degree_t standby_angle)
      : minimum_angle(min_angle),
        maximum_angle(max_angle),
        rest_angle(standby_angle),
        motor(joint_motor),
        mpu(accelerometer)
  {
  }

  /// Initialize the joint object, This must be called before any other
  /// function.
  void Initialize()
  {
    motor.Initialize();
    mpu.Initialize();
  }

  /// Move the motor to the (calibrated) angle desired.
  void SetPosition(float angle)
  {
    units::angle::degree_t angle_to_degrees(angle);
    units::angle::degree_t calibrated_angle =
        angle_to_degrees - zero_offset_angle;
    calibrated_angle =
        std::clamp(calibrated_angle, minimum_angle, maximum_angle);
    sjsu::LogInfo("%f", calibrated_angle.to<double>());
    motor.SetAngle(calibrated_angle);
  }

  /// Sets the zero_offset_angle value that the motor uses to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffset(float offset)
  {
    units::angle::degree_t offset_to_degrees(offset);
    zero_offset_angle = offset_to_degrees;
  }

  /// Return the acceleration values for the MPU6050 on the joint as a
  /// JointAcceleration of floats
  Acceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(mpu.Read());
    Acceleration acceleration;
    acceleration.x = static_cast<float>(acceleration_to_float.x);
    acceleration.y = static_cast<float>(acceleration_to_float.y);
    acceleration.z = static_cast<float>(acceleration_to_float.z);

    return acceleration;
  }

  void SetSpeed(float targetspeed)
  {
    float current_speed = speed_.to<float>();
    units::angular_velocity::revolutions_per_minute_t speed(
        std::lerp(current_speed, targetspeed, kLerpStep));

    speed_ = speed;
    motor.SetSpeed(speed_);
  }

  int GetSpeed()
  {
    return int(speed_);
  }

  int GetPosition()
  {
    return int(position_);
  }


// edits the min angle. Useful for specific robot modes where full traversal might not be a good idea.
  void SetMinAngle(float newmin)
  {
    units::angle::degree_t minimum_angle(newmin);
  }


// edits the min angle. Useful for specific robot modes where full traversal might not be a good idea.
void SetMaxAngle(float newmax)
{
    units::angle::degree_t maximum_angle(newmax);
}
  int getMinAngle()
  {
    return int(minimum_angle);
  }

  int GetMaxAngle()
  {
    return int(maximum_angle);
  }

//Changes bounds to what original values. Can be run on initialization to make sure there are no old configuartions in memory
  void ResetDefaultBounds()
  {
    units::angle::degree_t minimum_angle(0);
    units::angle::degree_t maximum_angle(180);

  }

 private:
  sjsu::RmdX & motor;
  sjsu::Mpu6050 & mpu;

  units::angle::degree_t minimum_angle     = 0_deg;
  units::angle::degree_t maximum_angle     = 180_deg;
  units::angle::degree_t rest_angle        = 0_deg;
  units::angle::degree_t zero_offset_angle = 0_deg;

  units::angular_velocity::revolutions_per_minute_t speed_     = 0_rpm;
  units::angular_velocity::revolutions_per_minute_t max_speed_ = 100_rpm;
  units::angle::degree_t position_                             = 0_deg;

  const float kLerpStep = .5;
};
}  // namespace sjsu::arm
