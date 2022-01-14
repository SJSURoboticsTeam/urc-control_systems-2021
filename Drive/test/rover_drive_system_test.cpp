// Reference Doc:
// https://docs.google.com/document/d/1GoI3ypr8xW_N2GrNbO3eNoYoz85498nix4rTv1WexoA/edit?usp=sharing
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

  const int kNonZero = 50;

  SECTION("1.1 should start with default values")
  {
    CHECK_EQ(drive.mc_data_.heartbeat_count, 0);
    CHECK_EQ(drive.mc_data_.is_operational, 0);
    CHECK_EQ(drive.mc_data_.drive_mode, 'S');
    CHECK_EQ(drive.mc_data_.rotation_angle, 0);
    CHECK_EQ(drive.mc_data_.speed, 0);
  }

  SECTION("2.1 should return the starting defaults response")
  {
    std::string expected_parameters =
        "?heartbeat_count=0&is_operational=0&drive_mode=S&battery=90"
        "&left_wheel_speed=0&left_wheel_angle=0&right_wheel_speed=0&right_"
        "wheel_angle=0&back_wheel_speed=0&back_wheel_angle=0";
    std::string actual_parameters = drive.GETParameters();
    CHECK_EQ(actual_parameters, expected_parameters);
  }

  SECTION("3.1 should parse expected values")
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
    CHECK_EQ(drive.mc_data_.heartbeat_count, 0);
    CHECK_EQ(drive.mc_data_.is_operational, 1);
    CHECK_EQ(drive.mc_data_.drive_mode, 'S');
    CHECK_EQ(drive.mc_data_.rotation_angle, 15);
    CHECK_EQ(drive.mc_data_.speed, 15);
  }

  SECTION("4.1 should return false at start")
  {
    CHECK_EQ(drive.mc_data_.is_operational, 0);
    CHECK_FALSE(drive.IsOperational());
  }

  SECTION("4.2 should return true after setting is_operational")
  {
    drive.mc_data_.is_operational = 1;
    CHECK(drive.IsOperational());
  }

  SECTION("5.1 should return starting drive mode 'S'")
  {
    CHECK_EQ(drive.GetCurrentMode(), 'S');
  }

  SECTION("6.1 should return false with same modes")
  {
    CHECK_EQ(drive.mc_data_.drive_mode, 'S');
    CHECK_EQ(drive.GetCurrentMode(), 'S');
    CHECK_FALSE(drive.IsNewMode());
  }

  SECTION("6.2 should return true with different modes")
  {
    drive.mc_data_.drive_mode = 'D';
    CHECK(drive.IsNewMode());
  }

  SECTION("7.1 should return true since rover is stopped at start")
  {
    CHECK(drive.IsStopped());
  }

  SECTION("7.2 should return false after all hub wheels start moving")
  {
    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("7.3 should return false if at least one hub wheel is moving")
  {
    drive.left_wheel_.SetHubSpeed(kNonZero);
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("8.1 should stop all wheels after one call")
  {
    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);
    drive.StopWheels();
    CHECK(drive.IsStopped());
  }

  SECTION("9.1 should lerp to 4 by setting speed to 2 → 3 → 4")
  {
    drive.SetWheelSpeed(4);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), 2);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), 2);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), 2);

    drive.SetWheelSpeed(4);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), 3);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), 3);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), 3);

    drive.SetWheelSpeed(4);
    CHECK_GE(drive.left_wheel_.GetHubSpeed(), 3);
    CHECK_GE(drive.right_wheel_.GetHubSpeed(), 3);
    CHECK_GE(drive.back_wheel_.GetHubSpeed(), 3);
  }

  SECTION("9.2 Should lerp from 4 to zero by setting speed to 2 → 1 → 0")
  {
    drive.left_wheel_.SetHubSpeed(4);
    drive.right_wheel_.SetHubSpeed(4);
    drive.back_wheel_.SetHubSpeed(4);

    drive.SetWheelSpeed(0);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), 2);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), 2);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), 2);

    drive.SetWheelSpeed(0);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), 1);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), 1);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), 1);

    drive.SetWheelSpeed(0);
    CHECK(drive.IsStopped());
  }

  SECTION("9.3 should lerp to -4 by setting speed to -2 → -3 → -4")
  {
    drive.SetWheelSpeed(-4);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), -2);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), -2);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), -2);

    drive.SetWheelSpeed(-4);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), -3);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), -3);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), -3);

    drive.SetWheelSpeed(-4);
    CHECK_GE(drive.left_wheel_.GetHubSpeed(), -4);
    CHECK_GE(drive.right_wheel_.GetHubSpeed(), -4);
    CHECK_GE(drive.back_wheel_.GetHubSpeed(), -4);
  }

  SECTION("9.4 Should lerp from -4 to zero by setting speed to -2 → -1 → -0")
  {
    drive.left_wheel_.SetHubSpeed(-4);
    drive.right_wheel_.SetHubSpeed(-4);
    drive.back_wheel_.SetHubSpeed(-4);

    drive.SetWheelSpeed(0);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), -2);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), -2);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), -2);

    drive.SetWheelSpeed(0);
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), -1);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), -1);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), -1);

    drive.SetWheelSpeed(0);
    CHECK(drive.IsStopped());
  }

  SECTION("10.1 should home wheels after moving")
  {
    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);
    drive.HomeWheels();
    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), 0);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 0);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 0);
  }

  SECTION("11.1 should return true when at start position")
  {
    CHECK(drive.AllWheelsAreHomed());
  }

  SECTION("11.2 should return false if all wheels are not homed")
  {
    drive.left_wheel_.SetSteerAngle(kNonZero);
    drive.right_wheel_.SetSteerAngle(kNonZero);
    drive.back_wheel_.SetSteerAngle(kNonZero);
    CHECK_FALSE(drive.AllWheelsAreHomed());
  }

  SECTION("11.3 should return false if one wheel is not homed")
  {
    drive.back_wheel_.SetSteerAngle(kNonZero);
    CHECK_FALSE(drive.AllWheelsAreHomed());
  }

  SECTION("12.1 should slow down when heartbeat is out of sync")
  {
    drive.mc_data_.heartbeat_count = kNonZero;
    drive.mc_data_.is_operational  = 1;

    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);
    drive.HandleRoverMovement();

    CHECK_LT(drive.left_wheel_.GetHubSpeed(), kNonZero);
    CHECK_LT(drive.right_wheel_.GetHubSpeed(), kNonZero);
    CHECK_LT(drive.back_wheel_.GetHubSpeed(), kNonZero);
  }

  SECTION("12.2 should stop movement when rover is not operational")
  {
    drive.mc_data_.is_operational = 0;

    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);
    drive.HandleRoverMovement();

    CHECK(drive.IsStopped());
  }

  SECTION("12.3 should switch into all the valid drive modes")
  {
    drive.mc_data_.is_operational = 1;

    drive.mc_data_.drive_mode = 'D';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'D');

    drive.mc_data_.drive_mode = 'T';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'T');

    drive.mc_data_.drive_mode = 'S';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'S');

    drive.mc_data_.drive_mode = 'B';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'B');

    drive.mc_data_.drive_mode = 'L';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'L');

    drive.mc_data_.drive_mode = 'R';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'R');
  }

  SECTION("12.4 should stay in the current mode when passed invalid drive mode")
  {
    drive.mc_data_.is_operational = 1;

    drive.mc_data_.drive_mode = 'Q';
    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'S');
  }

  SECTION("12.5 should stay in same mode when not operational but has new mode")
  {
    drive.mc_data_.is_operational = 0;
    drive.mc_data_.drive_mode     = 'D';

    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);

    drive.HandleRoverMovement();
    CHECK_EQ(drive.GetCurrentMode(), 'S');
  }

  SECTION("12.6 should only slow down when heartbeat not synced")
  {
    drive.mc_data_.heartbeat_count = kNonZero;
    drive.mc_data_.is_operational  = 0;
    drive.mc_data_.drive_mode      = 'D';

    drive.left_wheel_.SetHubSpeed(kNonZero);
    drive.right_wheel_.SetHubSpeed(kNonZero);
    drive.back_wheel_.SetHubSpeed(kNonZero);

    drive.HandleRoverMovement();

    CHECK_LT(drive.left_wheel_.GetHubSpeed(), kNonZero);
    CHECK_LT(drive.right_wheel_.GetHubSpeed(), kNonZero);
    CHECK_LT(drive.back_wheel_.GetHubSpeed(), kNonZero);
    CHECK_FALSE(drive.IsStopped());
    CHECK_EQ(drive.GetCurrentMode(), 'S');
  }

  SECTION("12.7.1 should have all steer motor angles at 90")
  {
    drive.mc_data_.is_operational = 1;
    drive.Initialize();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), 90);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 90);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 90);
  }

  SECTION("12.7.2 should have all hub motors at same non zero speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), drive.right_wheel_.GetHubSpeed());
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), drive.back_wheel_.GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.8.1 should clamp steer angles when over 45")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.rotation_angle = 1000;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), 12);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 45);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), -36);
  }

  SECTION("12.8.2 should clamp steer angles when under 45")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.rotation_angle = -1000;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), -45);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), -12);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 36);
  }

  SECTION("12.8.3 should have correct angles when set 10: L=6, R=-10, B=-12")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.rotation_angle = 10;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), 6);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 10);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), -14);
  }

  SECTION("12.8.4 should have correct angles when set -10: L=-10, R=-6, B=12")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.rotation_angle = -10;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), -10);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), -6);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 14);
  }

  SECTION("12.8.5 should get correct speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();
    // Currently the hub motors are all set to the same speed
    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), drive.right_wheel_.GetHubSpeed());
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), drive.back_wheel_.GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.8.6 should have correct starting angles L=-45, R=-135, B=90")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'D';

    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), -45);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), -135);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 90);
  }

  SECTION("12.9.1 should have correct starting angles")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'T';

    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetSteerAngle(), 0);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 60);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 110);
  }

  SECTION("12.9.2 should have all hub motors at same non zero speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'T';
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_EQ(drive.left_wheel_.GetHubSpeed(), drive.right_wheel_.GetHubSpeed());
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), drive.back_wheel_.GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.10.1 should adjust only the speed and angle of one wheel")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = 'L';
    drive.mc_data_.speed          = kNonZero;
    drive.mc_data_.rotation_angle = kNonZero;

    drive.HandleRoverMovement();
    drive.HandleRoverMovement();

    CHECK_NE(drive.left_wheel_.GetHubSpeed(), 0);
    CHECK_EQ(drive.right_wheel_.GetHubSpeed(), 0);
    CHECK_EQ(drive.back_wheel_.GetHubSpeed(), 0);

    CHECK_NE(drive.left_wheel_.GetSteerAngle(), 0);
    CHECK_EQ(drive.right_wheel_.GetSteerAngle(), 0);
    CHECK_EQ(drive.back_wheel_.GetSteerAngle(), 0);
  }
}
}  // namespace sjsu
