#include "L1_Peripheral/lpc40xx/can.hpp"
#include "L2_HAL/actuators/servo/rmd_x.hpp"
#include "utility/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

int main(void)
{
  sjsu::LogInfo("Starting the rover drive system...");

  sjsu::lpc40xx::Can can(sjsu::lpc40xx::Can::Channel::kCan2);
  sjsu::StaticAllocator<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);

  sjsu::lpc40xx::Can can_network(sjsu::lpc40xx::Can::Channel::kCan2);
  sjsu::RmdX left_hub_motor(can_network, 0x140);
  sjsu::RmdX left_steer_motor(can_network, 0x148);

  sjsu::RmdX right_hub_motor(can_network, 0x149);
  sjsu::RmDX right_steer_motor(can_network, 0x14A);

  sjsu::RmDX back_hub_motor(can_network, 0x14B);
  sjsu::RmDX back_steer_motor(can_network, 0x14C);


  sjsu::drive::Wheel left_wheel(left_hub_motor, left_steer_motor);
  sjsu::drive::Wheel back_wheel(left_hub_motor, left_steer_motor);
  sjsu::drive::Wheel right_wheel(left_hub_motor, left_steer_motor);

  sjsu::arm::RoverDriveSystem driveControl(left_wheel, right_wheel, back_wheel);

  // still need to include accelerometers, encoders, etc

  driveControl.Initialize();
  driveControl.Enable();
  driveControl.Home();

  while(true)
  {

  }

  return 0;
}
