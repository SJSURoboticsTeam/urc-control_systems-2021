#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/servo.hpp"
#include "Hand/finger.hpp"

namespace sjsu::arm
{
TEST_CASE("Finger Testing Case...")
{
    Mock<Servo> mock_servo;

    Finger finger(mock_servo.get());

    SECTION("Should return 0 when called...")
    {
        CHECK_EQ(finger.GetPosition(), 0);
    }
}
}//sjsu