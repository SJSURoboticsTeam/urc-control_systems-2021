#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "Interface/rover_system_interface.hpp"

namespace sjsu::common
{
TEST_CASE("Rover system testing")
{
  class TestSystem : public RoverSystemInterface
  {
    void Initialize(){};
    void PrintRoverData(){};
    std::string CreateGETRequestParameterWithRoverStatus()
    {
      return "";
    };
    void ParseMissionControlCommands(std::string & response)
    {
      response = "";
    };
    void HandleRoverCommands(){};
  };

  TestSystem rover;

  SECTION("1.1 should return zero heartbeat count")
  {
    CHECK_EQ(rover.GetHeartbeatCount(), 0);
  }
}
}  // namespace sjsu::common