#include "testing/testing_frameworks.hpp"

#include "devices/actuators/servo/rmd_x.hpp"

#include "../wheel.hpp"

namespace sjsu
{
DESCRIBE("Wheel")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));
  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  constexpr auto kId = 0x140;
  RmdX hub_motor(network, kId);
  RmdX steer_motor(network, kId);

  drive::Wheel mockWheel(hub_motor, steer_motor);
  mockWheel.Initialize();

  TEST_CASE("Get speed")
  {
    Mock<Wheel> mock;
    When(Method(mock, GetSpeed)).Return(5.0);
    Verify(Method(mock, GetSpeed));
  }
}
}  // namespace sjsu