#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "heartbeat.hpp"

namespace sjsu::common
{
TEST_CASE("Rover system testing")
{
  Heartbeat heartbeat;
  SECTION("1.1 should return default starting values")
  {
    CHECK_EQ(heartbeat.GetHeartbeatCount(), 0);
  }

  SECTION("2.1 should increment heartbeat count by 1")
  {
    heartbeat.IncrementHeartbeatCount();
    CHECK_EQ(heartbeat.GetHeartbeatCount(), 1);
  }

  SECTION("3.1 should verify heartbeat is synced at 0 and at 1")
  {
    CHECK(heartbeat.IsSyncedWithMissionControl(0));
    heartbeat.IncrementHeartbeatCount();
    CHECK(heartbeat.IsSyncedWithMissionControl(1));
  }

  SECTION("3.2 should verify heartbeat is out of sync")
  {
    CHECK_FALSE(heartbeat.IsSyncedWithMissionControl(100));
  }

  SECTION("3.3 should reset heartbeat to zero when out of sync")
  {
    heartbeat.IncrementHeartbeatCount();
    CHECK_NE(heartbeat.GetHeartbeatCount(), 0);
    heartbeat.IsSyncedWithMissionControl(0);
    CHECK_EQ(heartbeat.GetHeartbeatCount(), 0);
  }
}
}  // namespace sjsu::common