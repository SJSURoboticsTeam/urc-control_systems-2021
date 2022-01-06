#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

namespace sjsu
{
TEST_CASE("Drive system testing")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  RmdX left_steer_motor(network, 0x141);
  RmdX left_hub_motor(network, 0x142);
  RmdX right_steer_motor(network, 0x143);
  RmdX right_hub_motor(network, 0x144);
  RmdX back_steer_motor(network, 0x145);
  RmdX back_hub_motor(network, 0x146);

  Mock<Gpio> mock_wheel_homing_pin;
  InterruptCallback interrupt;
  Fake(Method(mock_wheel_homing_pin, Gpio::Read));
  Fake(Method(mock_wheel_homing_pin, Gpio::GetPin));
  Fake(Method(mock_wheel_homing_pin, Gpio::SetDirection));
  Fake(Method(mock_wheel_homing_pin, Gpio::ModuleInitialize));
  Fake(Method(mock_wheel_homing_pin, Gpio::DetachInterrupt));
  When(Method(mock_wheel_homing_pin, Gpio::AttachInterrupt))
      .AlwaysDo([&interrupt](InterruptCallback callback, Gpio::Edge) -> void
                { interrupt = callback; });

  drive::Wheel left_wheel("left", left_hub_motor, left_steer_motor,
                          mock_wheel_homing_pin.get());
  drive::Wheel right_wheel("right", right_hub_motor, right_steer_motor,
                           mock_wheel_homing_pin.get());
  drive::Wheel back_wheel("back", back_hub_motor, back_steer_motor,
                          mock_wheel_homing_pin.get());

  drive::RoverDriveSystem drive(left_wheel, right_wheel, back_wheel);

  SECTION("Checking default values")
  {
    CHECK(drive.mc_data_.heartbeat_count == 0);
    CHECK(drive.mc_data_.is_operational == 0);
    CHECK(drive.mc_data_.drive_mode == 'S');
    CHECK(drive.mc_data_.rotation_angle == 0);
    CHECK(drive.mc_data_.speed == 0);
  }

  SECTION("GETParameters is default values")
  {
    std::string expected_response =
        "?heartbeat_count=0&is_operational=0&drive_mode=S&battery=90"
        "&left_wheel_speed=0&left_wheel_angle=0&right_wheel_speed=0&right_"
        "wheel_angle=0&back_wheel_speed=0&back_wheel_angle=0";
    CHECK(expected_response == drive.GETParameters());
  }

  SECTION("ParseJSONResponse should return all values passed")
  {
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseJSONResponse(example_response);
    CHECK(drive.mc_data_.heartbeat_count == 0);
    CHECK(drive.mc_data_.is_operational == 1);
    CHECK(drive.mc_data_.drive_mode == 'S');
    CHECK(drive.mc_data_.rotation_angle == 15);
    CHECK(drive.mc_data_.speed == 15);
  }

  // SECTION("HandleRoverMovement should behave normally - 1st step")
  // {
  //   std::string example_response =
  //       "\r\n\r\n{\n"
  //       "  \"heartbeat_count\": 1,\n"
  //       "  \"is_operational\": 1,\n"
  //       "  \"drive_mode\": \"S\",\n"
  //       "  \"speed\": 15,\n"
  //       "  \"angle\": 15\n"
  //       "}";
  //   drive.ParseJSONResponse(example_response);
  //   drive.HandleRoverMovement();

  //   CHECK(drive.left_wheel_.GetHubSpeed() == 15);
  //   CHECK(drive.right_wheel_.GetHubSpeed() == 15);
  //   CHECK(drive.back_wheel_.GetHubSpeed() == 15);
  //   // Spin mode angles should not change from 90
  //   CHECK(drive.left_wheel_.GetSteerAngle() == 90);
  //   CHECK(drive.right_wheel_.GetSteerAngle() == 90);
  //   CHECK(drive.back_wheel_.GetSteerAngle() == 90);
  // }

  // SECTION("HandleRoverMovement should catch !is_operational - 2nd step")
  // {
  //   std::string example_response =
  //       "\r\n\r\n{\n"
  //       "  \"heartbeat_count\": 2,\n"
  //       "  \"is_operational\": 0,\n"s
  //       "  \"drive_mode\": \"S\",\n"
  //       "  \"speed\": 15,\n"
  //       "  \"angle\": 15\n"
  //       "}";
  //   drive.ParseJSONResponse(example_response);
  //   CHECK(drive.mc_data_.is_operational == 0);
  //   drive.HandleRoverMovement();
  //   // Should lerp down a speed before stopping
  //   CHECK(drive.left_wheel_.GetHubSpeed() != 15);
  //   CHECK(drive.right_wheel_.GetHubSpeed() != 15);
  //   CHECK(drive.back_wheel_.GetHubSpeed() != 15);
  //   // Should not be fully stopped yet
  //   CHECK(drive.left_wheel_.GetHubSpeed() > 0);
  //   CHECK(drive.right_wheel_.GetHubSpeed() > 0);
  //   CHECK(drive.back_wheel_.GetHubSpeed() > 0);
  //   // Spin mode angles should not change from 90
  //   CHECK(drive.left_wheel_.GetSteerAngle() == 90);
  //   CHECK(drive.right_wheel_.GetSteerAngle() == 90);
  //   CHECK(drive.back_wheel_.GetSteerAngle() == 90);
  // }
}
}  // namespace sjsu