#include "utility/log.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "peripherals/lpc40xx/can.hpp"
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
  sjsu::Mpu6050 rotunda_mpu(i2c, 0x68);
  sjsu::Mpu6050 shoulder_mpu(i2c, 0x69);
  sjsu::Mpu6050 elbow_mpu(i2c, 0x6A);
  sjsu::Mpu6050 wrist_mpu(i2c, 0x6B);

  // RMD addresses 0x141 - 0x148 are available
  sjsu::RmdX rotunda_motor(can_network, 0x141);
  sjsu::RmdX shoulder_motor(can_network, 0x142);
  sjsu::RmdX elbow_motor(can_network, 0x143);
  sjsu::RmdX left_wrist_motor(can_network, 0x144);
  sjsu::RmdX right_wrist_motor(can_network, 0x145);

  rotunda_motor.settings.gear_ratio     = 8;
  shoulder_motor.settings.gear_ratio    = 8;
  elbow_motor.settings.gear_ratio       = 8;
  left_wrist_motor.settings.gear_ratio  = 8;
  right_wrist_motor.settings.gear_ratio = 8;

  sjsu::arm::Joint rotunda(rotunda_motor, rotunda_mpu, 0_deg, 3600_deg,
                           1800_deg);
  sjsu::arm::Joint shoulder(shoulder_motor, shoulder_mpu);
  sjsu::arm::Joint elbow(elbow_motor, elbow_mpu);
  sjsu::arm::WristJoint wrist(left_wrist_motor, right_wrist_motor, wrist_mpu);
  sjsu::arm::RoverArmSystem arm(rotunda, shoulder, elbow, wrist);

  esp.Initialize();
  arm.Initialize();

  while (1)
  {
    try
    {
      sjsu::LogInfo("Making new request...");
      std::string endpoint = "arm?example=1&param=2";  // include status updates
      std::string response = esp.GETRequest(endpoint);
      sjsu::TimeoutTimer serverTimeout(5s);  // server has 5s timeout
      // Do stuff with arm here...
      sjsu::Delay(3s);
      if (serverTimeout.HasExpired())
      {
        sjsu::LogWarning("Server timed out! Reconnecting...");
        esp.ConnectToServer();
      }
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Uncaught error in main()!");
      if (!esp.IsConnected())
      {
        esp.ConnectToWifi();
        esp.ConnectToServer();
      }
    }
  }
}
