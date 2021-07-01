#include "testing/testing_frameworks.hpp"
#include "utility/log.hpp"

#include "../../Common/esp.hpp"

namespace sjsu
{
TEST_CASE("Testing ESP Wi-Fi Module")
{
  // Mock<common::Esp> esp;
  // Fake(Method(esp, common::Esp::Initialize));
  SECTION("should initialize esp wi-fi module")
  {
    esp.Initialize();
  }

  SECTION("should make get request to jsonplaceholder.typicode.com/todos/1")
  {
    esp.GETRequest("?example=3");
  }
}
}  // namespace sjsu