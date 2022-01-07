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
    double x;
    double y;
    double z;
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
  void SetPosition(double angle)
  {
    units::angle::degree_t angle_to_degrees(angle);
    units::angle::degree_t calibrated_angle =
        angle_to_degrees - zero_offset_angle;
    calibrated_angle = units::math::min(
        units::math::max(calibrated_angle, minimum_angle), maximum_angle);
    sjsu::LogInfo("%f", calibrated_angle.to<double>());
    motor.SetAngle(calibrated_angle);
  }

  /// Sets the zero_offset_angle value that the motor uses to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffset(double offset)
  {
    units::angle::degree_t offset_to_degrees(offset);
    zero_offset_angle = offset_to_degrees;
  }

  /// Return the acceleration values for the MPU6050 on the joint as a
  /// JointAcceleration of doubles
  Acceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_double(mpu.Read());
    Acceleration acceleration;
    acceleration.x = static_cast<double>(acceleration_to_double.x);
    acceleration.y = static_cast<double>(acceleration_to_double.y);
    acceleration.z = static_cast<double>(acceleration_to_double.z);

    return acceleration;
  }

  void SetSpeed(double targetspeed){
    double current_speed = speed_;
    speed_ = std::lerp(current_speed, targetspeed, kLerpStep);
    motor.SetSpeed(speed_);
  }

  int GetSpeed()
  {
    return int(speed_);
  }

  int GetPosition()
  {
    return 0;
  }

 private:
  units::angle::degree_t minimum_angle     = 0_deg;
  units::angle::degree_t maximum_angle     = 180_deg;
  units::angle::degree_t rest_angle        = 0_deg;
  units::angle::degree_t zero_offset_angle = 0_deg;
  sjsu::RmdX & motor;
  sjsu::Mpu6050 & mpu;
  const double kLerpStep = 0.5;
  units::angular_velocity::revolutions_per_minute_t speed_ = 0_rpm;
};
}  // namespace sjsu::arm
