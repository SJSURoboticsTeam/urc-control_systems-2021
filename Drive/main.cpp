#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

#include "RoverDriveSystems.hpp"

int main()
{
  char mode;
  float speed;
  units::angle::degree_t angle;

  sjsu::RoverDriveSystems roverDriver;
  roverDriver.Initialize();

  while (true)
  {
    if (mode == 'D')
      roverDriver.handleDriveMode(speed, angle);

    if (mode == 'T')
      roverDriver.handleTranslationMode(speed, angle);

    if (mode == 'S')
      roverDriver.handleSpinMode(speed, angle);
  }
  return 0;
}
