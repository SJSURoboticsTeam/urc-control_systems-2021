#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "utility/math/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"
#include "../../Common/esp.hpp"

int main(void)
{
  sjsu::LogInfo("Starting the rover drive system...");
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<2>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);
  sjsu::RmdX rmd_x6_steering(can_network, 0x141);
  sjsu::RmdX rmd_x6_hub_motor(can_network, 0x142);
  rmd_x6_steering.settings.gear_ratio  = 8;
  rmd_x6_hub_motor.settings.gear_ratio = 8;

  sjsu::drive::Wheel demoWheel(rmd_x6_steering, rmd_x6_hub_motor);
  sjsu::common::Esp esp;

  sjsu::LogInfo("Initializing wheel...");

  demoWheel.Initialize();
  esp.Initialize();
  // esp.isConnectedToWiFi();
  esp.GET("?example=3");

  // Not sure how to implement
  // sjsu::lpc40xx::Can can_network(sjsu::lpc40xx::Can::Channel::kCan2);
  // sjsu::RmdX left_hub_motor(can_network, 0x140);
  // address 0x140 - 0x148
  // sjsu::RmdX left_steer_motor(can_network, 0x148);
  // sjsu::drive::Wheel left_wheel(left_hub_motor, left_steer_motor);

  return 0;
}
