#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "utility/math/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

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

  sjsu::drive::Wheel demoWheel(rmd_x6_hub_motor, rmd_x6_steering);
  // sjsu::drive::Wheel demoWheel(rmd_x6_steering, rmd_x6_hub_motor);

  sjsu::LogInfo("Initializing wheel...");
  try
  {
    // steering motor demo
    demoWheel.Initialize();
    double currPos = demoWheel.GetPosition();
    sjsu::LogInfo("Current Position(start): %d", currPos);
    // demoWheel.SetSteeringAngle(180_deg);
    demoWheel.HomeWheel();
    sjsu::Delay(10s);
    currPos = demoWheel.GetPosition();
    sjsu::LogInfo("Current Position(end): %d", currPos);
  }
  catch (...)
  {
    sjsu::LogInfo("something broke");
  }

  // try
  // {
  //   // hub motor demo
  //   demoWheel.Initialize();
  //   double currSpeed = demoWheel.GetSpeed();
  //   sjsu::LogInfo("Current Speed(start): %d", currPos);
  //   demoWheel.SetHubSpeed(100_rpm);
  //   sjsu::Delay(1s);
  //   currSpeed = demoWheel.GetSpeed();
  //   sjsu::LogInfo("Current Speed(end): %d", currPos);
  //   demoWheel.SetHubSpeed(0_rpm);
  //   currSpeed = demoWheel.GetSpeed();
  // }
  // catch (...)
  // {
  //   sjsu::LogInfo("something broke");
  // }

  return 0;
}
