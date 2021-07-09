#include "testing/testing_frameworks.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "utility/log.hpp"
#include "utility/math/units.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

namespace sjsu
{
TEST_CASE("Testing Wheel")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  sjsu::RmdX rmd_wheel_left(network, 0x140);
  sjsu::RmdX rmd_steer_left(network, 0x141);

  sjsu::drive::Wheel wheel(rmd_wheel_left, rmd_steer_left);

  SECTION("should initialize wheel")
  {
    wheel.Initialize();
    CHECK(wheel.GetSpeed() == doctest::Approx(0.0));
    CHECK(wheel.GetPosition() == doctest::Approx(0.0));
    CHECK(wheel.kMaxPosSpeed == 100_rpm);
    CHECK(wheel.kMaxNegSpeed == -100_rpm);
  }

  SECTION("should set wheel speed and angle")
  {
    wheel.SetHubSpeed(50_rpm);
    wheel.SetSteeringAngle(20_deg);
    CHECK(wheel.GetSpeed() == doctest::Approx(50.0));
    CHECK(wheel.GetPosition() == doctest::Approx(20.0));

    wheel.SetHubSpeed(10_rpm);
    wheel.SetSteeringAngle(-20_deg);
    CHECK(wheel.GetSpeed() == doctest::Approx(10.0));
    CHECK(wheel.GetPosition() == doctest::Approx(0.0));
  }

  SECTION("should set extreme wheel speeds")
  {
    wheel.SetHubSpeed(150_rpm);
    CHECK(wheel.GetSpeed() == doctest::Approx(100.0));

    wheel.SetHubSpeed(-200_rpm);
    CHECK(wheel.GetSpeed() == doctest::Approx(-100.0));
  }

  SECTION("should set extreme wheel positions")
  {
    wheel.SetSteeringAngle(500_deg);
    CHECK(wheel.GetPosition() == doctest::Approx(360.0));

    wheel.SetSteeringAngle(-360_deg);  // resets wheel to zero
    wheel.SetSteeringAngle(-500_deg);
    CHECK(wheel.GetPosition() == doctest::Approx(-360.0));
  }

  SECTION("should home wheel positions")
  {
    wheel.HomeWheel();
    CHECK(wheel.homing_offset_angle_ == 0_deg);
  }
}
}  // namespace sjsu
