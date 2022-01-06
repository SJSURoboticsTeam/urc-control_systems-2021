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
    units::angle::degree_t calibrated_angle = angle_to_degrees - zero_offset_angle;
    calibrated_angle                        = units::math::min(
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

  /// Return the acceleration values for the MPU6050 on the joint as a JointAcceleration of doubles
  JointAcceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_double(mpu.Read());
    JointAcceleration acceleration;
    acceleration.y=static_cast<double>(acceleration_to_double.y);
    acceleration.y=static_cast<double>(acceleration_to_double.y);
    acceleration.z=static_cast<double>(acceleration_to_double.z);

    return acceleration;
  }

  void SetSpeed(double speed)
  {
  units::angular_velocity::revolutions_per_minute_t speed_to_rpm(speed);
  speed_ = speed_to_rpm;
  motor.SetSpeed(speed_);
  }

  private:
  // The minimum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t minimum_angle = 0_deg;
  // The maximum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t maximum_angle = 180_deg;
  // The angle the joint will move to when is not operational
  units::angle::degree_t rest_angle = 0_deg;
  // The angle between the motor's zero position and the actual homed zero
  // positon.
  units::angle::degree_t zero_offset_angle = 0_deg;
  // Motor object that controls the joint
  sjsu::RmdX & motor;
  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 & mpu;
  // speed of the joint motor
  units::angular_velocity::revolutions_per_minute_t speed_;
};
}  // namespace sjsu::arm
