#include "utility/log.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "peripherals/lpc17xx/pwm.hpp"
#include "utility/time/timeout_timer.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
#include "../../Common/esp.hpp"

int main()
{
  sjsu::LogInfo("Starting the rover arm system...");
  sjsu::common::Esp esp;
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<2>();
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);

  // Need to use an address translator
  sjsu::Mpu6050 rotunda_mpu(i2c, 0x66);
  sjsu::Mpu6050 shoulder_mpu(i2c, 0x67);
  sjsu::Mpu6050 elbow_mpu(i2c, 0x68);
  sjsu::Mpu6050 wrist_mpu(i2c, 0x69);

  // RMD addresses 0x141 - 0x148 are available
  sjsu::RmdX rotunda_motor(can_network, 0x141);
  sjsu::RmdX shoulder_motor(can_network, 0x142);
  sjsu::RmdX elbow_motor(can_network, 0x143);
  sjsu::RmdX left_wrist_motor(can_network, 0x144);
  sjsu::RmdX right_wrist_motor(can_network, 0x145);

  //PWM for the servo motors for fingers
  sjsu::lpc40xx::Pwm & pinky_pwm = sjsu::lpc40xx::GetPwm<1, 0>();
  sjsu::lpc40xx::Pwm & ring_pwm = sjsu::lpc40xx::GetPwm<1, 1>();
  sjsu::lpc40xx::Pwm & middle_pwm = sjsu::lpc40xx::GetPwm<1, 2>();
  sjsu::lpc40xx::Pwm & pointer_pwm = sjsu::lpc40xx::GetPwm<1, 3>();
  sjsu::lpc40xx::Pwm & thumb_pwm = sjsu::lpc40xx::GetPwm<1, 4>();

  //Servo Motors for fingers
  sjsu::Servo pinky_servo(pinky_pwm);
  sjsu::Servo ring_servo(ring_pwm);
  sjsu::Servo middle_servo(middle_pwm);
  sjsu::Servo pointer_servo(pointer_pwm);
  sjsu::Servo thumb_servo(thumb_pwm);

  rotunda_motor.settings.gear_ratio     = 8;
  shoulder_motor.settings.gear_ratio    = 8;
  elbow_motor.settings.gear_ratio       = 8;
  left_wrist_motor.settings.gear_ratio  = 8;
  right_wrist_motor.settings.gear_ratio = 8;

  sjsu::arm::ArmJoint rotunda(rotunda_motor, rotunda_mpu, 0, 3600,
                           1800);
  sjsu::arm::ArmJoint shoulder(shoulder_motor, shoulder_mpu);
  sjsu::arm::ArmJoint elbow(elbow_motor, elbow_mpu);
  sjsu::arm::WristJoint wrist(left_wrist_motor, right_wrist_motor, wrist_mpu);
  sjsu::arm::Finger pinky(pinky_servo);
  sjsu::arm::Finger ring(ring_servo);
  sjsu::arm::Finger middle(middle_servo);
  sjsu::arm::Finger pointer(pointer_servo);
  sjsu::arm::Finger thumb(thumb_servo);
  sjsu::arm::Hand hand(wrist, pinky, ring, middle, pointer, thumb);
  sjsu::arm::RoverArmSystem arm(rotunda, shoulder, elbow, hand);

  esp.Initialize();
  arm.Initialize();

  // Arm control loop
  // 1. Arm sys creates GET request parameters - returns endpoint+parameters
  // 2. Make GET request using esp - returns response body as string
  // 3. Arm sys parses GET response
  // 4. Arm sys handles arm movement

  while (1)
  {
    try
    {
      sjsu::LogInfo("Making new request now...");
      std::string endpoint = "arm" + arm.GETParameters();
      std::string response = esp.GET(endpoint);
      sjsu::TimeoutTimer serverTimeout(5s);  // server has 5s timeout
      arm.ParseJSONResponse(response);
      arm.HandleRoverMovement();
      arm.IncrementHeartbeatCount();
      arm.PrintRoverData();
      sjsu::Delay(3s);
      if (serverTimeout.HasExpired())
      {
        sjsu::LogWarning("Server timed out! Reconnecting...");
        esp.ConnectToServer();
      }
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Uncaught error in main() - Stopping Arm!");
      // Stop arm
      if (!esp.IsConnected())
      {
        esp.ConnectToWifi();
        esp.ConnectToServer();
      }
    }
    catch (const sjsu::arm::RoverArmSystem::ParseError &)
    {
      sjsu::LogError("Parsing Error: Arguments not equal");
    }
  }

  return 0;
}