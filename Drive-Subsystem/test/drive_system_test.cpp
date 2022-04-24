#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "drive_system.hpp"
#include "wheel.hpp"

namespace sjsu::drive
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

  RmdX motor(network, 0x141);

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

  drive::Wheel left_wheel("left", motor, motor, mock_wheel_homing_pin.get());
  drive::Wheel right_wheel("right", motor, motor, mock_wheel_homing_pin.get());
  drive::Wheel back_wheel("back", motor, motor, mock_wheel_homing_pin.get());
  drive::RoverDriveSystem::Wheels wheels = { &left_wheel, &right_wheel,
                                             &back_wheel };
  drive::RoverDriveSystem drive(wheels);

  const int kNonZero = 50;

  SECTION("1.1 should start with default values")
  {
    CHECK_EQ(drive.mc_data_.heartbeat_count, 0);
    CHECK_EQ(drive.mc_data_.is_operational, 0);
    CHECK_EQ(drive.mc_data_.wheel_shift, 0);
    CHECK_EQ(drive.mc_data_.drive_mode, RoverDriveSystem::Modes::SpinMode);
    CHECK_EQ(drive.mc_data_.rotation_angle, 0);
    CHECK_EQ(drive.mc_data_.speed, 0);
  }

  SECTION("2.1 should return the starting defaults response")
  {
    std::string expected_parameters =
        "?heartbeat_count=0&is_operational=0&wheel_shift=0&drive_mode=S&"
        "battery=0&left_wheel_speed=0&left_wheel_angle=0&right_wheel_speed=0&"
        "right_wheel_angle=0&back_wheel_speed=0&back_wheel_angle=0";
    std::string actual_parameters =
        drive.CreateGETRequestParameterWithRoverStatus();
    CHECK_EQ(expected_parameters, actual_parameters);
  }

  SECTION("3.1 should parse expected values")
  {
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"wheel_shift\": 0,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "}";
    drive.ParseMissionControlCommands(example_response);
    CHECK_EQ(drive.mc_data_.heartbeat_count, 0);
    CHECK_EQ(drive.mc_data_.is_operational, 1);
    CHECK_EQ(drive.mc_data_.wheel_shift, 0);
    CHECK_EQ(drive.mc_data_.drive_mode, RoverDriveSystem::Modes::SpinMode);
    CHECK_EQ(drive.mc_data_.rotation_angle, 15);
    CHECK_EQ(drive.mc_data_.speed, 15);
  }

  SECTION("3.2 should throw exception when given less than expected args")
  {
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "}";
    CHECK_THROWS(drive.ParseMissionControlCommands(example_response));
  }

  SECTION("3.3 should not throw exception when given more than expected args")
  {
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"wheel_shift\": 0,\n"
        "  \"drive_mode\": \"S\",\n"
        "  \"speed\": 15,\n"
        "  \"angle\": 15\n"
        "  \"misc\": 15\n"
        "}";
    CHECK_NOTHROW(drive.ParseMissionControlCommands(example_response));
  }

  SECTION("4.1 should return false at start")
  {
    CHECK_EQ(drive.mc_data_.is_operational, 0);
    CHECK_FALSE(drive.IsOperational(drive.mc_data_.is_operational));
  }

  SECTION("4.2 should return true after setting is_operational")
  {
    drive.mc_data_.is_operational = 1;
    CHECK(drive.IsOperational(drive.mc_data_.is_operational));
  }

  SECTION("5.1 should return starting drive mode 'S'")
  {
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);
  }

  SECTION("6.1 should return false with same modes")
  {
    CHECK_EQ(drive.mc_data_.drive_mode, RoverDriveSystem::Modes::SpinMode);
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);
    CHECK_FALSE(drive.IsNewMode());
  }

  SECTION("6.2 should return true with different modes")
  {
    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::DriveMode;
    CHECK(drive.IsNewMode());
  }

  SECTION("7.1 should return true since rover is stopped at start")
  {
    CHECK(drive.IsStopped());
  }

  SECTION("7.2 should return false after all hub wheels start moving")
  {
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("7.3 should return false if at least one hub wheel is moving")
  {
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("8.1 should stop all wheels after one call")
  {
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);
    drive.StopWheels();
    CHECK(drive.IsStopped());
  }

  SECTION("9.1 should lerp to 4 by setting speed to 2 → 3 → 4")
  {
    drive.SetWheelSpeed(4, 0);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 2);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 2);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), 2);

    drive.SetWheelSpeed(4, 0);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 3);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 3);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), 3);

    drive.SetWheelSpeed(4, 0);
    CHECK_GE(drive.wheels_.right_->GetHubSpeed(), 3);
    CHECK_GE(drive.wheels_.right_->GetHubSpeed(), 3);
    CHECK_GE(drive.wheels_.back_->GetHubSpeed(), 3);
  }

  SECTION("9.2 Should lerp from 4 to zero by setting speed to 2 → 1 → 0")
  {
    drive.wheels_.left_->SetHubSpeed(4);
    drive.wheels_.right_->SetHubSpeed(4);
    drive.wheels_.back_->SetHubSpeed(4);

    drive.SetWheelSpeed(0, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), 2);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 2);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), 2);

    drive.SetWheelSpeed(0, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), 1);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 1);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), 1);

    drive.SetWheelSpeed(0, 0);
    CHECK(drive.IsStopped());
  }

  SECTION("9.3 should lerp to -4 by setting speed to -2 → -3 → -4")
  {
    drive.SetWheelSpeed(-4, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), -2);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), -2);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), -2);

    drive.SetWheelSpeed(-4, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), -3);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), -3);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), -3);

    drive.SetWheelSpeed(-4, 0);
    CHECK_GE(drive.wheels_.left_->GetHubSpeed(), -4);
    CHECK_GE(drive.wheels_.right_->GetHubSpeed(), -4);
    CHECK_GE(drive.wheels_.back_->GetHubSpeed(), -4);
  }

  SECTION("9.4 Should lerp from -4 to zero by setting speed to -2 → -1 → -0")
  {
    drive.wheels_.left_->SetHubSpeed(-4);
    drive.wheels_.right_->SetHubSpeed(-4);
    drive.wheels_.back_->SetHubSpeed(-4);

    drive.SetWheelSpeed(0, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), -2);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), -2);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), -2);

    drive.SetWheelSpeed(0, 0);
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(), -1);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), -1);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), -1);

    drive.SetWheelSpeed(0, 0);
    CHECK(drive.IsStopped());
  }

  SECTION("10.1 should home wheels after moving")
  {

  }

  SECTION("11.1 should return true when at start position")
  {
    CHECK(drive.AllWheelsAreHomed());
  }

  SECTION("11.2 should return false if all wheels are not homed")
  {
    drive.wheels_.left_->SetSteerAngle(kNonZero);
    drive.wheels_.right_->SetSteerAngle(kNonZero);
    drive.wheels_.back_->SetSteerAngle(kNonZero);
    CHECK_FALSE(drive.AllWheelsAreHomed());
  }

  SECTION("11.3 should return false if one wheel is not homed")
  {
    drive.wheels_.back_->SetSteerAngle(kNonZero);
    CHECK_FALSE(drive.AllWheelsAreHomed());
  }

  SECTION("12.1 should slow down when heartbeat is out of sync")
  {
    drive.mc_data_.heartbeat_count = kNonZero;
    drive.mc_data_.is_operational  = 1;

    drive.wheels_.left_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);
    drive.HandleRoverCommands();

    CHECK_LT(drive.wheels_.left_->GetHubSpeed(), kNonZero);
    CHECK_LT(drive.wheels_.right_->GetHubSpeed(), kNonZero);
    CHECK_LT(drive.wheels_.back_->GetHubSpeed(), kNonZero);
  }

  SECTION("12.2 should stop movement when rover is not operational")
  {
    drive.mc_data_.is_operational = 0;

    drive.wheels_.left_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);
    drive.HandleRoverCommands();

    CHECK(drive.IsStopped());
  }

  SECTION("12.3 should switch into all the valid drive modes")
  {
    drive.mc_data_.is_operational = 1;

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::DriveMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::DriveMode);

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::TranslateMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::TranslateMode);

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::SpinMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::BackWheelMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::BackWheelMode);

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::LeftWheelMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::LeftWheelMode);

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::RightWheelMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::RightWheelMode);
  }

  SECTION("12.4 should stay in the current mode when passed invalid drive mode")
  {
    drive.mc_data_.is_operational = 1;

    drive.mc_data_.drive_mode = RoverDriveSystem::Modes::SpinMode;
    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);
  }

  SECTION("12.5 should stay in same mode when not operational but has new mode")
  {
    drive.mc_data_.is_operational = 0;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;

    drive.wheels_.left_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);

    drive.HandleRoverCommands();
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);
  }

  SECTION("12.6 should only slow down when heartbeat not synced")
  {
    drive.mc_data_.heartbeat_count = kNonZero;
    drive.mc_data_.is_operational  = 0;
    drive.mc_data_.drive_mode      = RoverDriveSystem::Modes::DriveMode;

    drive.wheels_.left_->SetHubSpeed(kNonZero);
    drive.wheels_.right_->SetHubSpeed(kNonZero);
    drive.wheels_.back_->SetHubSpeed(kNonZero);

    drive.HandleRoverCommands();

    CHECK_LT(drive.wheels_.left_->GetHubSpeed(), kNonZero);
    CHECK_LT(drive.wheels_.right_->GetHubSpeed(), kNonZero);
    CHECK_LT(drive.wheels_.back_->GetHubSpeed(), kNonZero);
    CHECK_FALSE(drive.IsStopped());
    CHECK_EQ(drive.GetCurrentMode(), RoverDriveSystem::Modes::SpinMode);
  }

  SECTION("12.7.1 should have all steer motor angles at 90")
  {
    drive.mc_data_.is_operational = 1;
    drive.Initialize();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), 90);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), 90);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 90);
  }

  SECTION("12.7.2 should have all hub motors at same non zero speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(),
             drive.wheels_.right_->GetHubSpeed());
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(),
             drive.wheels_.back_->GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.7.3 Testing when inner angle is -45...")
  {
    drive.mc_data_.is_operational = 1;
    
  }

  SECTION("12.8.1 should clamp steer angles when over 45")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.rotation_angle = 1000;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), 12);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), 45);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), -36);
  }

  SECTION("12.8.2 should clamp steer angles when under 45")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.rotation_angle = -1000;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), -45);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), -12);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 36);
  }

  SECTION("12.8.3 should have correct angles when set 10: L=6, R=-10, B=-12")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.rotation_angle = 10;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), 6);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), 10);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), -14);
  }

  SECTION("12.8.4 should have correct angles when set -10: L=-10, R=-6, B=12")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.rotation_angle = -10;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), -10);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), -6);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 14);
  }

  SECTION("12.8.5 should get correct speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();
    // Currently the hub motors are all set to the same speed
    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(),
             drive.wheels_.right_->GetHubSpeed());
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(),
             drive.wheels_.back_->GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.8.6 should have correct starting angles L=-45, R=-135, B=90")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::DriveMode;

    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), -45);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), -135);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 90);
  }

  SECTION("12.9.1 should have correct starting angles")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::TranslateMode;

    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetSteerAngle(), 0);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), 60);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 110);
  }

  SECTION("12.9.2 should have all hub motors at same non zero speed")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::TranslateMode;
    drive.mc_data_.speed          = kNonZero;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_EQ(drive.wheels_.left_->GetHubSpeed(),
             drive.wheels_.right_->GetHubSpeed());
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(),
             drive.wheels_.back_->GetHubSpeed());
    CHECK_FALSE(drive.IsStopped());
  }

  SECTION("12.10.1 should adjust only the speed and angle of one wheel")
  {
    drive.mc_data_.is_operational = 1;
    drive.mc_data_.drive_mode     = RoverDriveSystem::Modes::LeftWheelMode;
    drive.mc_data_.speed          = kNonZero;
    drive.mc_data_.rotation_angle = kNonZero;

    drive.HandleRoverCommands();
    drive.HandleRoverCommands();

    CHECK_NE(drive.wheels_.left_->GetHubSpeed(), 0);
    CHECK_EQ(drive.wheels_.right_->GetHubSpeed(), 0);
    CHECK_EQ(drive.wheels_.back_->GetHubSpeed(), 0);

    CHECK_NE(drive.wheels_.left_->GetSteerAngle(), 0);
    CHECK_EQ(drive.wheels_.right_->GetSteerAngle(), 0);
    CHECK_EQ(drive.wheels_.back_->GetSteerAngle(), 0);
  }

  SECTION("13.1 should keep initial leg orientation")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(0);
    CHECK_EQ(original_left_wheel, drive.wheels_.left_);
    CHECK_EQ(original_right_wheel, drive.wheels_.right_);
    CHECK_EQ(original_back_wheel, drive.wheels_.back_);
  }

  SECTION("13.2 keep initial leg orientation with negative position")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(-1);
    CHECK_EQ(original_left_wheel, drive.wheels_.left_);
    CHECK_EQ(original_right_wheel, drive.wheels_.right_);
    CHECK_EQ(original_back_wheel, drive.wheels_.back_);
  }

  SECTION("13.3 keep initial leg orientation with position 3")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(3);
    CHECK_EQ(original_left_wheel, drive.wheels_.left_);
    CHECK_EQ(original_right_wheel, drive.wheels_.right_);
    CHECK_EQ(original_back_wheel, drive.wheels_.back_);
  }

  SECTION("13.4 should rotate wheel orientation by one leg")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(1);
    CHECK_FALSE(original_left_wheel == drive.wheels_.left_);
    CHECK_FALSE(original_right_wheel == drive.wheels_.right_);
    CHECK_FALSE(original_back_wheel == drive.wheels_.back_);
  }

  SECTION("13.4 should rotate wheel orientation by two leg")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(2);
    CHECK_FALSE(original_left_wheel == drive.wheels_.back_);
    CHECK_FALSE(original_right_wheel == drive.wheels_.left_);
    CHECK_FALSE(original_back_wheel == drive.wheels_.right_);
  }

  SECTION("13.4 should rotate wheel orientation by two leg")
  {
    drive::Wheel * original_left_wheel  = drive.wheels_.left_;
    drive::Wheel * original_right_wheel = drive.wheels_.right_;
    drive::Wheel * original_back_wheel  = drive.wheels_.back_;
    drive.SwitchLegOrientation(5);
    CHECK_FALSE(original_left_wheel == drive.wheels_.back_);
    CHECK_FALSE(original_right_wheel == drive.wheels_.left_);
    CHECK_FALSE(original_back_wheel == drive.wheels_.right_);
  }
  SECTION(
      "14.1 should return greater than 100% when voltage is greater than 4.2")
  {
  }
  SECTION("14.2 should return a warning when voltage is lower than 3.05V") {}
  SECTION("14.2 should return a low battery warning at voltage = 3.05V") {}
  SECTION("14.3 should return another warning when voltage is lower than 3.25V")
  {
  }
  SECTION("14.3.1 Shouldn’t return a warning at voltage = 3.25V") {}
  SECTION(
      "14.4 Should return the percentage of the useable battery remains at any "
      "value")
  {
  }
}
}  // namespace sjsu::drive
