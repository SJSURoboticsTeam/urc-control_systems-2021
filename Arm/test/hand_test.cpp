#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
#include "Hand/wrist_joint.hpp"
#include "Hand/hand.hpp"

namespace sjsu
{
    TEST_CASE("Hand system testing")
    {
        Mock<Can> mock_can;
        Fake(Method(mock_can, Can::ModuleInitialize));
        Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
        Fake(Method(mock_can, Can::Receive));
        Fake(Method(mock_can, Can::HasData));
    }
}