#include "./RoverDriveSystems.hpp"

int main()
{
  string mode;
  float speed;
  units::angle::degree_t angle;

  RoverDriveSystems roverDriver;
  roverDriver.Initialize();

      while (true)
  {
    // {speed, angle, mode } = getMissionControlCommands()

    if (mode == "drive")
      roverDriver.handleDriveMode(speed, angle);

    if (mode == "translation")
      roverDriver.handleTranslationMode(speed, angle);

    if (mode == "spin")
      roverDriver.handleSpinMode(speed, angle);
  }
}