#include "utility/log.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "utility/time/timeout_timer.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "Drive-Subsystem/wheel.hpp"
#include "../Common/esp.hpp"
#include "../Common/esp_v2.hpp"
#include "drive_system.hpp"

int main(void)
{
  sjsu::lpc40xx::SetMaximumClockSpeed();
  sjsu::LogInfo("Starting the rover drive system...");
  sjsu::common::Esp esp;
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<2>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);

  // RMD addresses 0x141 - 0x148 are available
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

  sjsu::Gpio & left_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<0, 15>();
  sjsu::Gpio & right_wheel_homing_pin = sjsu::lpc40xx::GetGpio<2, 9>();
  sjsu::Gpio & back_wheel_homing_pin  = sjsu::lpc40xx::GetGpio<0, 18>();

  sjsu::drive::Wheel left_wheel("left", left_hub_motor, left_steer_motor,
                                left_wheel_homing_pin);
  sjsu::drive::Wheel right_wheel("right", right_hub_motor, right_steer_motor,
                                 right_wheel_homing_pin);
  sjsu::drive::Wheel back_wheel("back", back_hub_motor, back_steer_motor,
                                back_wheel_homing_pin);
  sjsu::drive::RoverDriveSystem::Wheels wheels = { &left_wheel, &right_wheel,
                                                   &back_wheel };
  sjsu::drive::RoverDriveSystem drive(wheels);

  sjsu::LogInfo("Initializing esp and drive system...");
  esp.Initialize();
  drive.Initialize();
  sjsu::Delay(5s);

  while(true)
  {
    //   sjsu::logInfo("Homing wheels...");
    //   drive.HomeWheels();
    //   sjsu::Delay(5s);

      sjsu::LogInfo("Setting wheel speed to 10RPMS for 5 seconds...");
      drive.SetWheelSpeed(10.0);
      sjsu::Delay(5s);

      sjsu::logInfo("Setting angle of wheels to ");

  }