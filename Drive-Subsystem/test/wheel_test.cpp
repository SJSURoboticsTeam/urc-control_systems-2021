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
  SECTION("1.1 should call the initialize function twice")
  {
    wheel.Initialize();
    Verify(Method(mock_can, ModuleInitialize)).Twice();
  }

  SECTION("1.2 should start with default values")
  {
    CHECK(wheel.GetName() == "wheel");
    CHECK(wheel.GetHubSpeed() == 0);
    CHECK(wheel.GetSteerAngle() == 0);
    CHECK(wheel.GetHomingOffset() == 0);
  }

  SECTION("2.1 should set hub speed within min and max range")
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

  SECTION("2.2 should set hub speed beyond the min and max range")
  {
    wheel.SetHubSpeed(150);
    CHECK(wheel.GetHubSpeed() == 100);

    wheel.SetHubSpeed(-150);
    CHECK(wheel.GetHubSpeed() == -100);
  }

  SECTION("3.1 should set steer angle within min and max range")
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

  SECTION(
      "3.2 should set mod 360 of steer angle  when beyond min and max range")
  {
    wheel.SetSteerAngle(400);
    CHECK(wheel.GetSteerAngle() == 40);

    wheel.SetSteerAngle(-400);
    CHECK(wheel.GetSteerAngle() == -40);
  }
}
}  // namespace sjsu
