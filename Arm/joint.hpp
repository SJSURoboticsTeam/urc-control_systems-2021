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
      : motor_(joint_motor), mpu_(accelerometer){};

  Joint(sjsu::RmdX & joint_motor,
        sjsu::Mpu6050 & accelerometer,
        float min_angle,
        float max_angle,
        float standby_angle)
      : motor_(joint_motor),
        mpu_(accelerometer),
        kMinimumAngle(min_angle),
        kMaximumAngle(max_angle),
        kRestAngle(standby_angle){};

  void Initialize()
  {
    motor_.Initialize();
    mpu_.Initialize();
  }

  /// Move the motor to the (calibrated) angle desired.
  void SetPosition(float angle)
  {
    angle += offset_angle_;
    position_ = std::clamp(angle, kMinimumAngle, kMaximumAngle);
    units::angle::degree_t angle_to_degrees(position_);
    motor_.SetAngle(angle_to_degrees);
  }

  /// Sets the offset angle so motor can find its true zero
  void SetZeroOffset(float offset)
  {
    offset_angle_ = offset;
  }

  /// Return the acceleration values from the MPU6050
  Acceleration GetAccelerometerData()
  {
    sjsu::Accelerometer::Acceleration_t acceleration_to_float(mpu_.Read());
    Acceleration acceleration;
    acceleration.x = static_cast<float>(acceleration_to_float.x);
    acceleration.y = static_cast<float>(acceleration_to_float.y);
    acceleration.z = static_cast<float>(acceleration_to_float.z);

    return acceleration;
  }

  void SetJointSpeed(double targetspeed){
    speed_ = SetLerpSpeed(targetspeed)
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

  int GetOffsetAngle()
  {
    return int(offset_angle_);
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
  sjsu::RmdX & motor_;
  sjsu::Mpu6050 & mpu_;

  float offset_angle_ = 0;
  float speed_        = 0;
  float position_     = 0;

  const float kLerpStep     = .5;
  const float kMinimumAngle = 0;
  const float kMaximumAngle = 180;
  const float kRestAngle    = 0;
  const float kMaxSpeed     = 100;
};
}  // namespace sjsu::arm
