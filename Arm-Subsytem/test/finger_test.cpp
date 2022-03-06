#include "testing/testing_frameworks.hpp"
#include "Hand/finger.hpp"

namespace sjsu::arm
{
TEST_CASE("Finger Testing Case...")
{
  Finger finger(0);

  SECTION("1.1 Should start with default values")
  {
    CHECK_EQ(finger.GetPosition(), 0);
    CHECK_EQ(finger.GetMaxAngle(), 180);
    CHECK_EQ(finger.GetMinAngle(), 0);
    CHECK_EQ(finger.GetPWM(), 0_us);
    CHECK_EQ(finger.GetPwmPin(), 0);
  }

  SECTION("2.1 Should set position to 0 when set to 0")
  {
    finger.SetPositionAndPwm(0);
    CHECK_EQ(finger.GetPosition(), 0);
  }

  SECTION("2.2 Should set position to 0 when set to -1")
  {
    finger.SetPositionAndPwm(-1);
    CHECK_EQ(finger.GetPosition(), 0);
  }

  SECTION("2.3 Should set position to 180 when set to 180")
  {
    finger.SetPositionAndPwm(180);
    CHECK_EQ(finger.GetPosition(), 180);
  }

  SECTION("2.4 Should set position to 180 when set to 181")
  {
    finger.SetPositionAndPwm(181);
    CHECK_EQ(finger.GetPosition(), 180);
  }

  SECTION("3.1 Should set speed to 10 when set to 10")
  {
    finger.SetSpeed(10);
    CHECK_EQ(finger.GetSpeed(), 10);
  }
}
}  // namespace sjsu::arm