#include "utility/log.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "utility/time/timeout_timer.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "wheel.hpp"
#include "../Common/esp.hpp"
#include "rover_drive_system.hpp"

int main(void)
{
  

  // Drive control loop
  // 1. Drive sys creates GET request parameters - returns endpoint+parameters
  // 2. Make GET request using esp - returns response body as string
  // 3. Drive sys parses GET response
  // 4. Drive sys handles rover movement - move or switch driving modes

  while (1)
  {
    try
    {
      sjsu::common::StateOfCharge st;
      sjsu::Delay(1000ms);
      sjsu::LogInfo("Max: %f", st.StateOfCharge_MAX());
     // esp.IsServerExpired(serverTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Uncaught error in main() - Stopping Rover!");
      //drive.SetWheelSpeed(0);
      // if (!esp.IsConnected())
      // {
      //  // esp.ConnectToWifi();
      //  // esp.ConnectToServer();
      // }
    }
  }

  return 0;
}