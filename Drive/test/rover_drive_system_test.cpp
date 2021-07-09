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

  sjsu::RmdX left_steer_motor(network, 0x141);
  sjsu::RmdX left_hub_motor(network, 0x142);
  sjsu::RmdX right_steer_motor(network, 0x143);
  sjsu::RmdX right_hub_motor(network, 0x144);
  sjsu::RmdX back_steer_motor(network, 0x145);
  sjsu::RmdX back_hub_motor(network, 0x146);

  left_steer_motor.settings.gear_ratio  = 8;
  left_hub_motor.settings.gear_ratio    = 8;
  right_steer_motor.settings.gear_ratio = 8;
  right_hub_motor.settings.gear_ratio   = 8;
  back_steer_motor.settings.gear_ratio  = 8;
  back_hub_motor.settings.gear_ratio    = 8;

  sjsu::drive::Wheel left_wheel(left_hub_motor, left_steer_motor);
  sjsu::drive::Wheel right_wheel(right_hub_motor, right_steer_motor);
  sjsu::drive::Wheel back_wheel(back_hub_motor, back_steer_motor);

  sjsu::drive::RoverDriveSystem drive_system(left_wheel, right_wheel,
                                             back_wheel);

  SECTION("Initialize()")
  {
    drive_system.Initialize();
    CHECK(drive_system.mc_data.is_operational == 1);
  }

  SECTION("should correctly create GET request with current data")
  {
    std::string expectedParam =
        "drive?is_operational=1&drive_mode=S&battery=90&left_wheel_speed=0."
        "000000&left_wheel_angle=0.000000&right_wheel_speed=0.000000&right_"
        "wheel_angle=0.000000&back_wheel_speed=0.000000&back_wheel_angle=0."
        "000000";
    std::string reqParam = drive_system.CreateRequestParameters();
    CHECK(reqParam == expectedParam);
  }

  SECTION("should parse mission control response")
  {
    std::string_view response =
        R"({"is_operational": 1, "drive_mode": "D", "speed": 10.0, "angle": 10.0})";
    drive_system.ParseJSONResponse(response);
    CHECK(drive_system.mc_data.is_operational == 1);
    CHECK(drive_system.mc_data.drive_mode == 'D');
    CHECK(drive_system.mc_data.speed == doctest::Approx(10.0));
    CHECK(drive_system.mc_data.rotation_angle == doctest::Approx(10.0));
  }

  SECTION("should stop rover & reset wheel positions")
  {
    drive_system.HomeWheels();
    CHECK(drive_system.left_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.right_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.back_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.left_wheel_.GetPosition() == doctest::Approx(0.0));
    CHECK(drive_system.right_wheel_.GetPosition() == doctest::Approx(0.0));
    CHECK(drive_system.back_wheel_.GetPosition() == doctest::Approx(0.0));
  }

  SECTION("should set wheel speeds to 10_rpm")
  {
    drive_system.SetWheelSpeed(10_rpm);
    CHECK(drive_system.left_wheel_.GetSpeed() == doctest::Approx(10.0));
    CHECK(drive_system.right_wheel_.GetSpeed() == doctest::Approx(10.0));
    CHECK(drive_system.back_wheel_.GetSpeed() == doctest::Approx(10.0));
  }

  SECTION("should stop rover and set current_mode_ to drive")
  {
    CHECK(drive_system.GetCurrentMode() == 'S');
    char response[100] =
        R"({"is_operational": 1, "drive_mode": "D", "speed": 15.0, "angle": 20.0})";
    drive_system.ParseJSONResponse(response);
    drive_system.HandleRoverMovement();
    CHECK(drive_system.GetCurrentMode() == 'D');
    CHECK(drive_system.left_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.right_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.back_wheel_.GetSpeed() == doctest::Approx(0.0));
    CHECK(drive_system.left_wheel_.GetPosition() == doctest::Approx(-45.0));
    CHECK(drive_system.right_wheel_.GetPosition() == doctest::Approx(-135.0));
    CHECK(drive_system.back_wheel_.GetPosition() == doctest::Approx(90.0));
  }

  SECTION("should adjust rover speed to 15.0 and rotation angle to 20.0")
  {
    CHECK(drive_system.GetCurrentMode() != 'D');  // TODO: why isn't it drive?
    drive_system.HandleRoverMovement();
    drive_system.HandleRoverMovement();
    CHECK(drive_system.GetCurrentMode() == drive_system.mc_data.drive_mode);
    CHECK(drive_system.left_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(drive_system.right_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(drive_system.back_wheel_.GetSpeed() == doctest::Approx(15.0));
    CHECK(drive_system.left_wheel_.GetPosition() == doctest::Approx(-45.0));
    CHECK(drive_system.right_wheel_.GetPosition() == doctest::Approx(-135.0));
    CHECK(drive_system.back_wheel_.GetPosition() == doctest::Approx(110.0));
  }
}
}  // namespace sjsu