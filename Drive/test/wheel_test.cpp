#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "wheel.hpp"

namespace sjsu
{
TEST_CASE("Wheel testing")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  StaticMemoryResource<1024> memory;
  CanNetwork network(mock_can.get(), &memory);
  RmdX hub_motor(network, 0x140);
  RmdX steer_motor(network, 0x141);

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

  drive::Wheel wheel("wheel", hub_motor, steer_motor,
                     mock_wheel_homing_pin.get());

  // Mock<drive::Wheel> wheel;
  // Fake(Method(wheel, GetName)).AlwaysReturn("test");

  //   Mock<drive::Wheel> spy(wheel);
  //   When(Method(spy, GetSteerAngle)).AlwaysReturn(11);

  SECTION("Checking default values")
  {
    CHECK(wheel.GetName() == "wheel");
    CHECK(wheel.GetHubSpeed() == 0);
    CHECK(wheel.GetSteerAngle() == 0);
  }

  SECTION("Initializing wheel")
  {
    wheel.Initialize();
    Verify(Method(mock_can, ModuleInitialize)).Twice();
    CHECK(mock_can.get().CurrentSettings().baud_rate == 1_MHz);
  }

  SECTION("Setting hub speed to random normal speed")
  {
    int random_num = rand() % 100 + 1;
    wheel.SetHubSpeed(random_num);
    CHECK(wheel.GetHubSpeed() == random_num);

    random_num = (rand() % 100 - 1) * -1;
    wheel.SetHubSpeed(random_num);
    CHECK(wheel.GetHubSpeed() == random_num);

    wheel.SetHubSpeed(0);
    CHECK(wheel.GetHubSpeed() == 0);
  }

  SECTION("Setting hub speed beyond maximum speed")
  {
    wheel.SetHubSpeed(150);
    CHECK(wheel.GetHubSpeed() == 100);

    wheel.SetHubSpeed(-150);
    CHECK(wheel.GetHubSpeed() == -100);
  }

  SECTION("Setting steer angle to random normal angle")
  {
    int random_num = rand() % 360 + 1;
    wheel.SetSteerAngle(random_num);
    CHECK(wheel.GetSteerAngle() == random_num);

    random_num = (rand() % 360 - 1) * -1;
    wheel.SetSteerAngle(random_num);
    CHECK(wheel.GetSteerAngle() == random_num);

    wheel.SetSteerAngle(0);
    CHECK(wheel.GetSteerAngle() == 0);
  }

  SECTION("Setting steer angle beyond maximum angle")
  {
    wheel.SetSteerAngle(400);
    CHECK(wheel.GetSteerAngle() == 40);

    wheel.SetSteerAngle(-400);
    CHECK(wheel.GetSteerAngle() == -40);
  }

  SECTION("Spying on Wheel")
  {
    // TODO: Get spy logic working correctly - idk why it's not working...
    Mock<drive::Wheel> spy(wheel);
    When(Method(spy, GetSteerAngle)).AlwaysReturn(10);
    drive::Wheel & test = spy.get();
    CHECK(test.GetSteerAngle() == 10);
  }
}
}  // namespace sjsu
