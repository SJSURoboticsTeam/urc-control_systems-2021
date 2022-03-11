#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "Interface/rover_system_interface.hpp"

namespace sjsu::common
{
TEST_CASE("Rover system testing")
{
  class TestSystem : public RoverSystemInterface
  {
   public:
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

    RoverMissionControlData mc_data_;
  };

  TestSystem rover;

  SECTION("1.1 should verify default values")
  {
    CHECK_EQ(rover.mc_data_.is_operational, 0);
    CHECK_EQ(rover.mc_data_.heartbeat_count, 0);
  }

  SECTION("2.1 should verify rover is operational when passed 1")
  {
    CHECK(rover.IsOperational(1));
  }

  SECTION("2.2 should verify rover is not operational when passed 0")
  {
    CHECK_FALSE(rover.IsOperational(0));
  }

  SECTION("2.3 should verify rover is not operational when passed -1")
  {
    CHECK_FALSE(rover.IsOperational(-1));
  }

  SECTION("2.4 should verify rover is not operational when passed 2")
  {
    CHECK_FALSE(rover.IsOperational(2));
  }
}
}  // namespace sjsu::common