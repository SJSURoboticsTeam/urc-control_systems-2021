#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
// Testing if this was possible - more headaches than needed atm
namespace sjsu
{
auto MockCanNetwork()
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));
  return mock_can;
}
}  // namespace sjsu