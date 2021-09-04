#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "utility/time/timeout_timer.hpp"
#include "utility/math/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"
#include "../../Common/esp.hpp"

int main(void)
{
  // sjsu::lpc40xx::SetMaximumClockSpeed();

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

  // Slip ring GPIO pins
  // sjsu::Gpio & left_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<0, 15>();
  // sjsu::Gpio & right_wheel_homing_pin = sjsu::lpc40xx::GetGpio<2, 9>();
  // sjsu::Gpio & back_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<0, 18>();

  // Button GPIO pins
  sjsu::Gpio & left_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<1, 19>();
  sjsu::Gpio & right_wheel_homing_pin = sjsu::lpc40xx::GetGpio<1, 15>();
  sjsu::Gpio & back_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<0, 30>();

  sjsu::drive::Wheel left_wheel("left", left_hub_motor, left_steer_motor,
                                left_wheel_homing_pin);
  sjsu::drive::Wheel right_wheel("right", right_hub_motor, right_steer_motor,
                                 right_wheel_homing_pin);
  sjsu::drive::Wheel back_wheel("back", back_hub_motor, back_steer_motor,
                                back_wheel_homing_pin);
  sjsu::drive::RoverDriveSystem drive_system(left_wheel, right_wheel,
                                             back_wheel);

  sjsu::LogInfo("Initializing drive system...");
  drive_system.Initialize();
  // esp.Initialize();

  // Drive control loop
  // 1. Drive sys creates GET request parameters - returns endpoint+parameters
  // 2. Make GET request using esp - returns response body as string
  // 3. Drive sys parses GET response
  // 4. Drive sys handles rover movement - move or switch driving modes
  while (1)
  {
    drive_system.HomeWheels();
  }

  // while (1)
  // {
  //   try
  //   {
  //     sjsu::LogInfo("Making new request...");
  //     std::string parameters = drive_system.GETRequestParameters();
  //     std::string response   = esp.GETRequest(parameters);
  //     sjsu::TimeoutTimer serverTimeout(5s);  // server has 5s timeout
  //     drive_system.ParseJSONResponse(response);
  //     drive_system.HandleRoverMovement();
  //     drive_system.PrintRoverData();
  //     if (serverTimeout.HasExpired())
  //     {
  //       sjsu::LogWarning("Server timed out! Must reconnect!");
  //       esp.ConnectToServer();
  //     }
  //   }
  //   catch (const std::exception & e)
  //   {
  //     sjsu::LogError("Uncaught error in main() - Stopping Rover!");
  //     drive_system.SetWheelSpeed(0_rpm);
  //     if (!esp.IsConnected())
  //     {
  //       esp.ConnectToWifi();
  //       esp.ConnectToServer();
  //     }
  //   }
  // }

  return 0;
}
