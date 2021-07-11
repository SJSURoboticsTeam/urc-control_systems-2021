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
  sjsu::common::Esp esp;
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<2>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);

  // rmd addresses 0x141 - 0x148 are available
  sjsu::RmdX left_steer_motor(can_network, 0x141);
  sjsu::RmdX left_hub_motor(can_network, 0x142);
  sjsu::RmdX right_steer_motor(can_network, 0x143);
  sjsu::RmdX right_hub_motor(can_network, 0x144);
  sjsu::RmdX back_steer_motor(can_network, 0x145);
  sjsu::RmdX back_hub_motor(can_network, 0x146);

  left_steer_motor.settings.gear_ratio  = 8;
  left_hub_motor.settings.gear_ratio    = 8;
  right_steer_motor.settings.gear_ratio = 8;
  right_hub_motor.settings.gear_ratio   = 8;
  back_steer_motor.settings.gear_ratio  = 8;
  back_hub_motor.settings.gear_ratio    = 8;

  sjsu::drive::Wheel left_wheel(left_hub_motor, left_steer_motor);

  left_wheel.SetHubSpeed(10_rpm);
  // sjsu::drive::Wheel right_wheel(right_hub_motor, right_steer_motor);
  // sjsu::drive::Wheel back_wheel(back_hub_motor, back_steer_motor);

  // sjsu::drive::RoverDriveSystem drive_system(left_wheel, right_wheel,
  //                                            back_wheel);

  // sjsu::LogInfo("Initializing wheels and esp...");
  // esp.Initialize();
  // left_wheel.Initialize();
  // right_wheel.Initialize();
  // back_wheel.Initialize();

  // Drive control loop
  // 1. Drive sys creates GET request parameters - returns endpoint+params
  // 2. Make GET request using esp - returns response body in string_view
  // 3. Drive sys parses GET response
  // 4. Drive sys handles rover movement - may move or switch modes

  // while (true)
  // {
  //   try
  //   {
  //     std::string parameters    = drive_system.CreateRequestParameters();
  //     std::string_view response = esp.GETRequest(parameters);
  //     sjsu::LogInfo("Response Body:\n%s", response.data());
  //     drive_system.ParseJSONResponse(response);
  //     drive_system.HandleRoverMovement();
  //     drive_system.PrintRoverData();
  //   }
  //   catch (const std::exception & e)
  //   {
  //     sjsu::LogError("Error in main()!");
  //     throw e;
  //   }
  // }

  return 0;
}
