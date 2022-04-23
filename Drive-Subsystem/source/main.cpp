#include "utility/log.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "utility/time/timeout_timer.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "wheel.hpp"
#include "../Common/esp.hpp"
#include "drive_system.hpp"

int main(void)
{
  sjsu::lpc40xx::SetMaximumClockSpeed();
  sjsu::LogInfo("Starting the rover drive system...");
  sjsu::LogInfo("You are here...");
  sjsu::common::Esp esp;
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<1>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);

  // RMD addresses 0x141 - 0x148 are available
  sjsu::RmdX left_steer_motor(can_network, 0x141);
  sjsu::RmdX left_hub_motor(can_network, 0x142);
  sjsu::RmdX right_steer_motor(can_network, 0x143);
  sjsu::RmdX right_hub_motor(can_network, 0x144);
  sjsu::RmdX back_steer_motor(can_network, 0x145);
  sjsu::RmdX back_hub_motor(can_network, 0x146);

  left_steer_motor.settings.gear_ratio  = 6;
  left_hub_motor.settings.gear_ratio    = 15;
  right_steer_motor.settings.gear_ratio = 6;
  right_hub_motor.settings.gear_ratio   = 15;
  back_steer_motor.settings.gear_ratio  = 6;
  back_hub_motor.settings.gear_ratio    = 15;

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
  // esp.Initialize();
  drive.Initialize();
  drive.mc_data_.is_operational = 1;

  // sjsu::Delay();
  sjsu::LogInfo("Homing wheels...");
  drive.HomeWheels();
  sjsu::LogInfo("Right wheel %f", drive.wheels_.right_->GetHomingOffset());
  sjsu::Delay(5s);

  while (true)
  {
    // sjsu::LogInfo("Setting wheel speed to 50 RPMS for 5 seconds...");
    // drive.wheels_.right_->SetHubSpeed(50.0);
    // drive.wheels_.left_->SetHubSpeed(50.0);
    // drive.wheels_.back_->SetHubSpeed(50.0);
    // sjsu::Delay(5s);
    // sjsu::LogInfo("Setting wheel speed to 0 RPMS for 5 seconds...");
    // drive.wheels_.right_->SetHubSpeed(0);
    // drive.wheels_.left_->SetHubSpeed(0);
    // drive.wheels_.back_->SetHubSpeed(0);
    // sjsu::Delay(5s);
    // sjsu::LogInfo("Setting wheel speed to -50 RPMS for 5 seconds...");
    // drive.wheels_.right_->SetHubSpeed(-50.0);
    // drive.wheels_.left_->SetHubSpeed(-50.0);
    // drive.wheels_.back_->SetHubSpeed(-50.0);
    // sjsu::Delay(5s);

    // sjsu::LogInfo("Spinning steer wheels to 20 degrees...");
    // drive.wheels_.left_->SetSteerAngle(20);
    // drive.wheels_.right_->SetSteerAngle(20);
    // drive.wheels_.back_->SetSteerAngle(20);
    // sjsu::Delay(5s);
    // sjsu::LogInfo("Spinning steer wheel to 0 degrees..");
    // drive.wheels_.left_->SetSteerAngle(0);
    // drive.wheels_.right_->SetSteerAngle(0);
    // drive.wheels_.back_->SetSteerAngle(0);
    // sjsu::Delay(5s);

    // sjsu::LogInfo("Spinning steer wheels to 20 degrees...");
    // drive.wheels_.left_->SetSteerAngle(20);
    // drive.wheels_.right_->SetSteerAngle(20);
    // drive.wheels_.back_->SetSteerAngle(20);
    // sjsu::LogInfo("Spinning speed to 0 rpms..");
    // drive.wheels_.left_->SetHubSpeed(0);
    // drive.wheels_.right_->SetHubSpeed(0);
    // drive.wheels_.back_->SetHubSpeed(0);
    // sjsu::Delay(5s);

    // sjsu::LogInfo("Spinning steer wheels to 0 degrees...");
    // drive.wheels_.left_->SetSteerAngle(0);
    // drive.wheels_.right_->SetSteerAngle(0);
    // drive.wheels_.back_->SetSteerAngle(0);
    // sjsu::LogInfo("Spinning speed to 50 rpms...");
    // drive.wheels_.left_->SetHubSpeed(50);
    // drive.wheels_.right_->SetHubSpeed(50);
    // drive.wheels_.back_->SetHubSpeed(50);
    // sjsu::Delay(5s);
    // int i, j;
    drive.mc_data_.drive_mode =
        sjsu::drive::RoverDriveSystem::Modes::RightWheelMode;
    drive.mc_data_.rotation_angle = 45;
    drive.mc_data_.speed          = 20;
    drive.HandleRoverCommands();
    sjsu::Delay(5s);
    drive.mc_data_.rotation_angle = -45;
    drive.mc_data_.speed          = 20;
    drive.HandleRoverCommands();

    //   for(i = 0; i<=40; i += 10){
    sjsu::LogInfo("Setting `D` mode waiting 5s...");
    drive.mc_data_.drive_mode = sjsu::drive::RoverDriveSystem::Modes::DriveMode;
    drive.HandleRoverCommands();
    sjsu::Delay(5s);
    // drive.mc_data_.speed = 3;
    drive.mc_data_.rotation_angle = -45;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    drive.mc_data_.rotation_angle = -37;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    drive.mc_data_.rotation_angle = 37;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    drive.mc_data_.rotation_angle = -25;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    drive.mc_data_.rotation_angle = 5;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    drive.mc_data_.rotation_angle = 0;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    // drive.mc_data_.speed = 3;
    drive.mc_data_.rotation_angle = 45;
    drive.HandleRoverCommands();
    sjsu::Delay(4s);
    //   }

    // drive.mc_data_.speed = -1;
    //   for(j = i; i>=-40; i -= 10){
    //     sjsu::LogInfo("Setting `D` mode waiting 5s...");
    //     drive.mc_data_.rotation_angle = i;
    //     drive.mc_data_.drive_mode =
    //     sjsu::drive::RoverDriveSystem::Modes::DriveMode;
    //     drive.HandleRoverCommands();
    //     sjsu::Delay(4s);
    //   }

    // sjsu::LogInfo("Setting `T` mode waiting 5s...");
    // drive.mc_data_.drive_mode =
    //     sjsu::drive::RoverDriveSystem::Modes::TranslateMode;
    // drive.HandleRoverCommands();
    // sjsu::Delay(5s);
  }
}
