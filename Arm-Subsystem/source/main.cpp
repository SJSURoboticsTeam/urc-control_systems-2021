#include "utility/log.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "peripherals/lpc17xx/pwm.hpp"
#include "utility/time/timeout_timer.hpp"
#include "arm_system.hpp"
#include "../../Common/esp.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

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

  sjsu::Pca9685 pca(i2c, 0x70);

  sjsu::arm::Finger pinky(4);
  sjsu::arm::Finger ring(3);
  sjsu::arm::Finger middle(2);
  sjsu::arm::Finger pointer(1);
  sjsu::arm::Finger thumb(0);

  rotunda_motor.settings.gear_ratio     = 8;
  shoulder_motor.settings.gear_ratio    = 8*16/65;  //gear ratio of motor times gear ratio of shoulder
  elbow_motor.settings.gear_ratio       = 8*2/5;        //gear ratio of motor times gear ratio of elbow
  left_wrist_motor.settings.gear_ratio  = 8;
  right_wrist_motor.settings.gear_ratio = 8;

  sjsu::arm::ArmJoint rotunda(rotunda_motor, rotunda_mpu, 0, 3600, 1800);
  sjsu::arm::ArmJoint shoulder(shoulder_motor, shoulder_mpu);
  sjsu::arm::ArmJoint elbow(elbow_motor, elbow_mpu);
  sjsu::arm::Arm arm(rotunda, shoulder, elbow);
  sjsu::arm::WristJoint wrist(left_wrist_motor, right_wrist_motor, wrist_mpu);

  sjsu::arm::Hand hand(pca, wrist, pinky, ring, middle, pointer, thumb);
  sjsu::arm::RoverArmSystem arm_system(arm, hand);

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
      std::string endpoint =
          "arm" + arm_system.CreateGETRequestParameterWithRoverStatus();
      std::string response = esp.GET(endpoint);
      sjsu::TimeoutTimer server_timeout(5s);  // server has 5s timeout
      arm_system.ParseMissionControlCommands(response);
      arm_system.HandleRoverCommands();
      // arm_system.IncrementHeartbeatCount();
      arm_system.PrintRoverData();
      sjsu::Delay(3s);
      esp.ReconnectIfServerTimedOut(server_timeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Uncaught error in main() - Stopping Arm!");
      // Stop arm
      if (!esp.IsConnected())
      {
        esp.ConnectToWifi();
        esp.ConnectToWebServer();
      }
    }
    catch (const sjsu::arm::RoverArmSystem::ParseError &)
    {
      sjsu::LogError("Parsing Error: Arguments not equal");
    }
  }

  return 0;
}