#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/servo.hpp"
#include "Hand/finger.hpp"

namespace sjsu::arm
{
TEST_CASE("Finger Testing Case...")
{
  Mock<Servo> mock_servo;

  Finger finger(mock_servo.get());

  SECTION("1.1 Should return 0 when finger is first initialized")
  {
    CHECK_EQ(finger.GetPosition(), 0);
  }

  SECTION("2.1 Boundry testing the upper bounds")
  {
    int max_angle_ = 180;
    
    CHECK_EQ(finger.GetPosition(), 0);
    finger.SetPosition(181);
    CHECK_EQ(finger.GetPosition(), 180);
  }
  SECTION("2.2 Boundry testing the upper bounds")
  {
    int min_angle_ = 0;
    CHECK_EQ(finger.GetPosition(), 0);
    finger.SetPosition(-1);
    CHECK_EQ(finger.GetPosition(), 0);
  }
}
}  // namespace sjsu::arm