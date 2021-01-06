#include "RoverArmSystem.hpp"

int main()
{
  sjsu::RoverArmSystem armControl;
  armControl.Initialize();
  armControl.Home();
  while (true)
  {
    armControl.Get_data();
    armControl.Move_arm();
  }
}
