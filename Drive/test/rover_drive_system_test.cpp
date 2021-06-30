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
    CHECK(driveSystem.mc_data.is_operational == 1);
  }

  SECTION("should correctly create GET request with current data")
  {
    std::string expectedParam =
        "drive?is_operational=1&drive_mode=S&battery=90&left_wheel_speed=0."
        "000000&left_wheel_angle=0.000000&right_wheel_speed=0.000000&right_"
        "wheel_angle=0.000000&back_wheel_speed=0.000000&back_wheel_angle=0.0";
    std::string reqParam = driveSystem.CreateGETParameters();
    CHECK(reqParam == expectedParam);
  }

  SECTION("should parse mission control response")
  {
    char response[100] =
        R"({"is_operational": 1, "drive_mode": "D", "speed": 10.0, "angle": 10.0})";
    driveSystem.ParseGETResponse(response);
    CHECK(driveSystem.mc_data.is_operational == 1);
    CHECK(driveSystem.mc_data.drive_mode == 'D');
    CHECK(driveSystem.mc_data.speed == doctest::Approx(10.0));
    CHECK(driveSystem.mc_data.rotation_angle == doctest::Approx(10.0));
  }

  SECTION("should stop rover & reset wheel positions")
  {
    driveSystem.Reset();
    CHECK(driveSystem.left_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.right_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.back_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.left_wheel_.GetPosition() == doctest::Approx(0.0));
    CHECK(driveSystem.right_wheel_.GetPosition() == doctest::Approx(0.0));
    CHECK(driveSystem.back_wheel_.GetPosition() == doctest::Approx(0.0));
  }

  SECTION("should set wheel speeds to 10_rpm")
  {
    driveSystem.SetWheelSpeed(10_rpm);
    CHECK(driveSystem.left_wheel_.GetSpeed() == doctest::Approx(10.0));
    CHECK(driveSystem.right_wheel_.GetSpeed() == doctest::Approx(10.0));
    CHECK(driveSystem.back_wheel_.GetSpeed() == doctest::Approx(10.0));
  }

  SECTION("should stop rover and set current_mode_ to drive")
  {
    CHECK(driveSystem.GetCurrentMode() == 'S');
    char response[100] =
        R"({"is_operational": 1, "drive_mode": "D", "speed": 15.0, "angle": 20.0})";
    driveSystem.ParseGETResponse(response);
    driveSystem.HandleRoverMovement();
    CHECK(driveSystem.GetCurrentMode() == 'D');
    CHECK(driveSystem.left_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.right_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.back_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(driveSystem.left_wheel_.GetPosition() == doctest::Approx(-45.0));
    CHECK(driveSystem.right_wheel_.GetPosition() == doctest::Approx(-135.0));
    CHECK(driveSystem.back_wheel_.GetPosition() == doctest::Approx(90.0));
  }

  SECTION("should adjust rover speed to 15.0 and rotation angle to 20.0")
  {
    CHECK(driveSystem.GetCurrentMode() != 'D');  // TODO: why isn't it drive?
    driveSystem.HandleRoverMovement();
    driveSystem.HandleRoverMovement();
    CHECK(driveSystem.GetCurrentMode() == driveSystem.mc_data.drive_mode);
    CHECK(driveSystem.left_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(driveSystem.right_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(driveSystem.back_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(driveSystem.left_wheel_.GetPosition() == doctest::Approx(-45.0));
    CHECK(driveSystem.right_wheel_.GetPosition() == doctest::Approx(-135.0));
    CHECK(driveSystem.back_wheel_.GetPosition() == doctest::Approx(110.0));
  }
}
}  // namespace sjsu