#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/servo.hpp"
#include "Hand/finger.hpp"

namespace sjsu::arm
{
TEST_CASE("Finger Testing Case...")
{
  Mock<Servo> mock_servo;

  Finger finger(mock_servo.get());

  SECTION("Should return 0 when finger is first initialized")
  {
    CHECK_EQ(finger.GetPosition(), 0);
  }

  SECTION("Should return 0, 10, 45, 90, 100 when SetPosition() is called")
  {
    CHECK_EQ(finger.GetPosition(), 0);
    finger.SetPosition(10);
    CHECK_EQ(finger.GetPosition(), 10);
    finger.SetPosition(45);
    CHECK_EQ(finger.GetPosition(), 45);
    finger.SetPosition(90);
    CHECK_EQ(finger.GetPosition(), 90);
    finger.SetPosition(100);
    CHECK_EQ(finger.GetPosition(), 100);
    finger.SetPosition(0);
    CHECK_EQ(finger.GetPosition(), 0);
  }

  SECTION("Boundry testing the angles")
  {
    max_angle_ = 180;
    min_angle_ = 0;
    CHECK_EQ(finger.GetPosition(), 0);
    finger.SetPosition(-1);
    CHECK_EQ(finger.GetPosition(), 0);
    finger.SetPosition(181);
    CHECK_EQ(finger.GetPosition(), 180);
  }
}
}  // namespace sjsu::arm