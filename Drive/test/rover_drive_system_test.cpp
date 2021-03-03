#include "devices/actuators/servo/rmd_x.hpp"

#include "testing/testing_frameworks.hpp"

namespace sjsu
{
TEST_CASE("Drive System")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));
  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  constexpr auto kId = 0x140;
  RmdX rmd_left_1(network, kId);
  SECTION("setup")
  {
    rmd_left_1.Initialize();
  }
}
}  // namespace sjsu