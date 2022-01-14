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
        float min_angle,
        float max_angle,
        float standby_angle)
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
    float calibrated_angle =
        angle + zero_offset_angle;
    calibrated_angle =
        std::clamp(calibrated_angle, minimum_angle, maximum_angle);
    position_ = calibrated_angle;
    sjsu::LogInfo("%f", calibrated_angle);
    units::angle::degree_t calibrated_angle_to_degrees(calibrated_angle);
    motor.SetAngle(calibrated_angle_to_degrees);
  }

  /// Sets the zero_offset_angle value that the motor uses to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffset(float offset)
  {
    zero_offset_angle = offset;
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
    float current_speed = speed_;
    float speed = std::lerp(current_speed, targetspeed, kLerpStep);

    speed_ = speed;
    units::angular_velocity::revolutions_per_minute_t speed_to_rpm(speed);
    motor.SetSpeed(speed_to_rpm);
  }

  int GetSpeed()
  {
    return int(speed_);
  }

  int GetPosition()
  {
    return int(position_);
  }

  int GetOffsetAngle()
  {
    return int(zero_offset_angle);
  }

 private:
  sjsu::RmdX & motor;
  sjsu::Mpu6050 & mpu;

  float minimum_angle     = 0;
  float maximum_angle     = 180;
  float rest_angle        = 0;
  float zero_offset_angle = 0;

  float speed_            = 0;
  float max_speed_        = 100;
  float position_         = 0;

  const float kLerpStep   = .5;
};
}  // namespace sjsu::arm
