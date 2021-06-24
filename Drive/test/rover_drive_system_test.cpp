#include "testing/testing_frameworks.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "utility/log.hpp"
#include "utility/math/units.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

namespace sjsu
{
TEST_CASE("Testing Drive System")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  sjsu::RmdX rmd_wheel_left(network, 0x140);
  sjsu::RmdX rmd_wheel_right(network, 0x142);
  sjsu::RmdX rmd_wheel_back(network, 0x144);
  sjsu::RmdX rmd_steer_left(network, 0x141);
  sjsu::RmdX rmd_steer_right(network, 0x143);
  sjsu::RmdX rmd_steer_back(network, 0x145);

  sjsu::drive::Wheel wheel_left(rmd_wheel_left, rmd_steer_left);
  sjsu::drive::Wheel wheel_right(rmd_wheel_right, rmd_steer_right);
  sjsu::drive::Wheel wheel_back(rmd_wheel_back, rmd_steer_back);
  sjsu::drive::RoverDriveSystem driveSystem(wheel_left, wheel_right,
                                            wheel_back);

  SECTION("Initialize()")
  {
    driveSystem.Initialize();
    CHECK(driveSystem.mission_control_data_.is_operational == 1);
  }

  SECTION("Switch to drive mode")
  {
    driveSystem.SetMode('D');
    driveSystem.ExchangeMissionControlData();
    driveSystem.Move();
    CHECK(static_cast<char>(driveSystem.current_mode_) == 'D');
  }

  SECTION("Switch to translation mode")
  {
    driveSystem.SetMode('T');
    driveSystem.ExchangeMissionControlData();
    driveSystem.Move();
    CHECK(static_cast<char>(driveSystem.current_mode_) == 'T');
  }

  SECTION("Switch to spin mode")
  {
    driveSystem.SetMode();
    driveSystem.ExchangeMissionControlData();
    driveSystem.Move();
    CHECK(static_cast<char>(driveSystem.current_mode_) == 'S');
  }
}

}  // namespace sjsu
   // strcpy(driveSystem.mission_control_data_.response_body,
   //        R"({"is_operational": 1, "drive_mode": "S", "speed": 0, "angle":
   //        0})");
   // strcpy(driveSystem.mission_control_data_.response_body,
   //        R"({"is_operational": 1, "drive_mode": "T", "speed": 0, "angle":
   //        0})");
   // strcpy(driveSystem.mission_control_data_.response_body,
   //        R"({"is_operational": 1, "drive_mode": "D", "speed": 0, "angle":
   //        0})");