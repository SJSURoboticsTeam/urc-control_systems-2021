#include "testing/testing_frameworks.hpp"

#include "../../Common/esp.hpp"
#include "utility/log.hpp"
// TODO - Need to add mocks, segfaults tests
namespace sjsu
{
TEST_CASE("Testing ESP Wi-Fi Module")
{
  common::Esp esp;
  SECTION("should initialize esp wi-fi module")
  {
    esp.Initialize();
    // CHECK(esp.espInitialized_ == true);
  }

  SECTION("should make get request to jsonplaceholder.typicode.com/todos/1")
  {
    esp.GET("?example=3");
  }
}
}  // namespace sjsu