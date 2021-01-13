#include "L1_Peripheral/lpc40xx/can.hpp"
#include "L2_HAL/actuators/servo/rmd_x.hpp"
#include "utility/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

int main(void)
{
  sjsu::LogInfo("Starting the rover drive system...");
  // Not sure how to implement
  // sjsu::lpc40xx::Can can_network(sjsu::lpc40xx::Can::Channel::kCan2);
  // sjsu::RmdX left_hub_motor(can_network, 0x140);
  // sjsu::RmdX left_steer_motor(can_network, 0x148);
  // sjsu::drive::Wheel left_wheel(left_hub_motor, left_steer_motor);

  return 0;
}
