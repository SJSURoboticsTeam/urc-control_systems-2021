#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "rover_drive_system.hpp"
#include "rover_drive_system_helper.hpp"
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

  Mock<drive::Wheel> spy_left_motor(left_wheel);
  Mock<drive::Wheel> spy_right_motor(right_wheel);
  Mock<drive::Wheel> spy_back_motor(back_wheel);

  Fake(Method(spy_left_motor, Wheel::HomeWheel));
  Fake(Method(spy_right_motor, Wheel::HomeWheel));
  Fake(Method(spy_back_motor, Wheel::HomeWheel));

  drive::Wheel & spied_left_motor  = spy_left_motor.get();
  drive::Wheel & spied_right_motor = spy_right_motor.get();
  drive::Wheel & spied_back_motor  = spy_back_motor.get();

  drive::RoverDriveSystem drive(spied_left_motor, spied_right_motor,
                                spied_back_motor);

  SECTION("checking default values")
  {
    CHECK(drive.mc_data_.heartbeat_count == 0);
    CHECK(drive.mc_data_.is_operational == 0);
    CHECK(drive.mc_data_.drive_mode == 'S');
    CHECK(drive.mc_data_.rotation_angle == 0);
    CHECK(drive.mc_data_.speed == 0);
    drive.Initialize();
  }

  SECTION("should create default values")
  {
    std::string expected_response =
        "?heartbeat_count=0&is_operational=0&drive_mode=S&battery=90"
        "&left_wheel_speed=0&left_wheel_angle=0&right_wheel_speed=0&right_"
        "wheel_angle=0&back_wheel_speed=0&back_wheel_angle=0";
    CHECK(expected_response == drive.GETParameters());
  }

  SECTION("should return all values passed")
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
    CHECK(drive.GetHeartbeatCount() == 0);
    CHECK(drive.mc_data_.heartbeat_count == 0);
    CHECK(drive.mc_data_.is_operational == 1);
    CHECK(drive.mc_data_.drive_mode == 'S');
    CHECK(drive.mc_data_.rotation_angle == 15);
    CHECK(drive.mc_data_.speed == 15);
  }

  SECTION("should increment heartbeat counter after HandleRoverMovement")
  {
    CHECK(drive.GetHeartbeatCount() == 0);
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseJSONResponse(example_response);
    drive.HandleRoverMovement();
    CHECK(drive.GetHeartbeatCount() == 1);
    example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 1,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseJSONResponse(example_response);
    drive.HandleRoverMovement();
    CHECK(drive.GetHeartbeatCount() == 2);
    example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 2,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseJSONResponse(example_response);
    drive.HandleRoverMovement();
    CHECK(drive.GetHeartbeatCount() == 3);
  }
  SECTION("should increment heartbeat counter after HandleRoverMovement")
  {
    CHECK(drive.GetHeartbeatCount() == 0);
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseJSONResponse(example_response);
    drive.HandleRoverMovement();
    CHECK(drive.GetHeartbeatCount() == 1);
  }
}
}  // namespace sjsu