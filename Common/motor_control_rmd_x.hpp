/*
 *  Modified by: Joshua Ramayrat
 *
 */


#pragma once

#include <algorithm>

#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "utility/time/time.hpp"
#include "utility/math/units.hpp"
#include "utility/log.hpp"
#include "peripherals/lpc40xx/can.hpp"

#define PI 3.14159265358979323846

namespace sjsu::common
{

// I made this general motor class to implement angular position, velocity, and acceleration sensing
// with information from the accelerometer and the RMD-X motors.

class Motor
{
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

 public:

    /*
     * These data structures consist of position and velocity
     * for all 3 axes and they could probably be placed into
     * another more general program instead of the joint header file.
     * 
     * DISCLAIMER: This may not actually be in degrees per second
     * because the timer has units of nano-seconds. Maybe this
     * needs to be fixed.
     * 
     */
    struct Velocity_t
    {
        units::angular_velocity::degrees_per_second theta_dot_x;
        units::angular_velocity::degrees_per_second theta_dot_y;
        units::angular_velocity::degrees_per_second theta_dot_z;

        void Print()
        {
            sjsu::LogInfo("{  x: %.4f deg/t,  y: %.4f deg/t,  z: %.4f deg/t }",
                            theta_dot_x.to<double>(),
                            theta_dot_y.to<double>(),
                            theta_dot_z.to<double>());
        }
    };

    struct Position_t
    {
        units::angle::degree theta_x;
        units::angle::degree theta_y;
        units::angle::degree theta_z;
        void Print()
        {
            sjsu::LogInfo("{  x: %.4f deg,  y: %.4f deg,  z: %.4f deg }",
                            theta_x.to<double>(),
                            theta_y.to<double>(),
                            theta_z.to<double>());
        }
    };


  Motor(sjsu::RmdX & joint_motor, sjsu::Mpu6050 & accelerometer)
      : motor(joint_motor), mpu(accelerometer)
  {
  }

  Motor(sjsu::RmdX & joint_motor,
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
  void SetPosition(units::angle::degree_t angle)
  {
  
    units::angle::degree_t calibrated_angle = angle - zero_offset_angle;
    
    calibrated_angle = units::math::min(units::math::max(calibrated_angle, minimum_angle), maximum_angle);

    sjsu::LogInfo("%f", calibrated_angle.to<double>());
    
    motor.SetAngle(calibrated_angle);
  
  }

  /// Sets the zero_offset_angle value that the motor uses to know its true '0'
  /// position. Called by RoverArmSystem::Home
  void SetZeroOffset(units::angle::degree_t offset)
  {
    zero_offset_angle = offset;
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  sjsu::Accelerometer::Acceleration_t GetAccelerometerData()
  {
    auto acc_all = mpu.Read();

    acc_x = acc_all.x
    acc_y = acc_all.y
    acc_z = acc_all.z


    return mpu.Read();
  }


  /*
   *  
   *  The goal of this function is to return the velocity of 
   *  the joint at a given interval of time. I'm integrating
   *  acceleration once from the accelerometer data to get velocity.
   *  I am using the timer library of the SJSU-Dev2 firmware platform.
   * 
   *  Disclaimer: This will accumulate measurement errors over time. 
   * 
   */
  Velocity_t GetAngularVelocity_1(sjsu::Accelerometer::Acceleration_t accel_input)
  {

      Velocity_t ang_velo;

      float initial_time = DefaultUptime();

      sjsu::Accelerometer::Acceleration_t accel_ti = accel_input;

      // Casting pre-defined acceleration unit types into
      // floats. I think there will be some clash if I
      // do computations with mis-matching datatypes but I
      // may be wrong.
      float accel_xi = static_cast<float>(accel_ti.x);
      float accel_yi = static_cast<float>(accel_ti.y);
      float accel_zi = static_cast<float>(accel_ti.z);

      sjsu::Accelerometer::Acceleration_t accel_tf = mpu.Read();

      float accel_xf = static_cast<float>(accel_tf.x);
      float accel_yf = static_cast<float>(accel_tf.y);
      float accel_zf = static_cast<float>(accel_tf.z);


      // I'm getting the next current time without running a wait command
      // so this may impact the integral computations.
      float final_time = DefaultUptime();

      float dt = final_time - initial_time;

      float ang_velo_x = (accel_xf - accel_xi)/dt;
      float ang_velo_y = (accel_yf - accel_yi)/dt;
      float ang_velo_z = (accel_zf - accel_zi)/dt;


      // Converting the original angular velocity computations 
      // from floats to the pre-defined unit datatypes.
      units::angular_velocity::degrees_per_second x_conv = ang_velo_x;
      units::angular_velocity::degrees_per_second y_conv = ang_velo_y;
      units::angular_velocity::degrees_per_second z_conv = ang_velo_z;

      ang_velo.theta_dot_x = x_conv;
      ang_velo.theta_dot_y = y_conv
      ang_velo.theta_dot_z = z_conv

      return ang_velo;
  }

  /*
   *  The goal of this function is to return the angular
   *  position of the object in degrees. 
   * 
   *  Disclaimer: This will accumulate measurement errors over time.
   * 
   */
  Position_t GetAngularPosition_1(Velocity_t velo_input)
  {
      Position_t ang_pos;

      float initial_time = DefaultUptime();

      Velocity_t velo_i = velo_input;
      float velo_xi = static_cast<float>(velo_i.theta_dot_x);
      float velo_yi = static_cast<float>(velo_i.theta_dot_y);
      float velo_zi = static_cast<float>(velo_i.theta_dot_z);

      sjsu::Accelerometer::Acceleration_t accel_current = mpu.Read();
      Velocity_t velo_tf = GetAngularVelocity_1(accel_current);

      float velo_xf = static_cast<float>(velo_tf.theta_dot_x);
      float velo_yf = static_cast<float>(.theta_dot_y);
      float velo_zf = static_cast<float>(velo_tf.theta_dot_z);


      // I'm getting the next current time without running a wait command
      // so this may impact the integral computations.
      float final_time = DefaultUptime();

      float dt = final_time - initial_time;


      float theta_x = (velo_xf - velo_xi)/dt;
      float theta_y = (velo_yf - velo_yi)/dt;
      float theta_z = (velo_zf - velo_zi)/dt;

      units::angle::degree theta_x_curr = theta_x;
      units::angle::degree theta_y_curr = theta_y;
      units::angle::degree theta_z_curr = theta_z;
      
      ang_pos.theta_x = theta_x_curr;
      ang_pos.theta_y = theta_y_curr;
      ang_pos.theta_z = theta_z_curr;

      return ang_pos;

  }

  Position_t GetAngularPosition_2(Acceleration_t accel_input)
  {

    Position_t returnPos;

    sjsu::Accelerometer::Acceleration_t accel = accel_input;

    float a_x = static_cast<float>(accel.x);
    float a_y = static_cast<float>(accel.y);
    float a_z = static_cast<float>(accel.z);



    /* DISCLAIMER: I'm not sure if this is going to work because I'm not sure
     * about what inputs/data types that 'units::math::atan' accepts.
     * 
     * Reference: https://www.delsys.com/downloads/USERSGUIDE/emgworks/HTMLDocuments/filteraccelerationasinclinationangle.htm
     */
    units::angle::radian_t rad_x = units::math::atan(a_x / (units::math::sqrt(units::math::pow(a_y, 2) + units::math::pow(a_z, 2))))
    units::angle::radian_t rad_y = units::math::atan(a_y / (units::math::sqrt(units::math::pow(a_x, 2) + units::math::pow(a_z, 2))))
    units::angle::radian_t rad_z = units::math::atan( ( units::math::sqrt( units::math::pow(a_x, 2) + units::math::pow(a_y, 2) ) / a_z ) )

    // degree = 1 rad * 180 / pi
    float deg_x = static_cast<float>(rad_x) * 180 / PI;
    float deg_y = static_cast<float>(rad_y) * 180 / PI;
    float deg_z = static_cast<float>(rad_z) * 180 / PI;

    units::angle::degree th_x = deg_x;
    units::angle::degree th_x = deg_x;
    units::angle::degree th_x = deg_x;

    returnPos.theta_x = th_x;
    returnPos.theta_y = th_y;
    returnPos.theta_z = th_z;

  
  }


};
}  // namespace sjsu::arm
