#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "rover_system_interface.hpp"

namespace sjsu::common
{
TEST_CASE("Rover system testing")
{
  class TestSystem : public RoverSystemInterface
  {
    void Initialize(){};
    void PrintRoverData(){};
    std::string GETParameters()
    {
      return "";
    };
    void ParseJSONResponse(std::string & response)
    {
      response = "";
    };
    void HandleRoverMovement(){};
  };

  TestSystem rover;

  SECTION("1.1 should return zero heartbeat count")
  {
    CHECK_EQ(rover.GetHeartbeatCount(), 0);
  }

  SECTION("2.1 should increment heartbeat count one time")
  {
    rover.IncrementHeartbeatCount();
    CHECK_EQ(rover.GetHeartbeatCount(), 1);
  }

  SECTION("3.1 should verify rover is synced at start")
  {
    CHECK(rover.IsHeartbeatSynced(0));
  }

  SECTION("3.2 should verify heartbeat not synced at start")
  {
    CHECK_FALSE(rover.IsHeartbeatSynced(1));
  }

  SECTION("3.3 should verify heartbeat is reset when not synced")
  {
    rover.IncrementHeartbeatCount();
    CHECK_EQ(rover.GetHeartbeatCount(), 1);
    CHECK_FALSE(rover.IsHeartbeatSynced(0));
    CHECK_EQ(rover.GetHeartbeatCount(), 0);
  }
}
}  // namespace sjsu::common