#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
#include "joint.hpp"

namespace sjsu
{
TEST_CASE("Joint system testing")
{
  auto accel_value = units::acceleration::meters_per_second_squared_t(5);
  Accelerometer::Acceleration_t example_acceleration = { accel_value,
                                                         accel_value,
                                                         accel_value };

  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);
  RmdX motor(network, 0x142);

  Mock<I2c> mock_i2c;
  Fake(Method(mock_i2c, I2c::ModuleInitialize));

  Mpu6050 mpu(mock_i2c.get(), 0x69);
  Mock<Mpu6050> mock_mpu(mpu);
  Fake(Method(mock_mpu, Mpu6050::ModuleInitialize));
  When(Method(mock_mpu, Mpu6050::Read)).AlwaysReturn(example_acceleration);
  Mpu6050 & spyed_mpu = mock_mpu.get();

  arm::Joint joint(motor, spyed_mpu);

  SECTION("should initialize and return default values")
  {
    joint.Initialize();
    CHECK(joint.GetSpeed() == 0);
    CHECK(joint.GetPosition() == 0);
  }

  SECTION("should boundary test setting the motor angle")
  {
    joint.SetPosition(5);
    // CHECK(joint.GetPosition() == 5);
    joint.SetPosition(180);
    joint.SetPosition(181);
    joint.SetPosition(0);
    joint.SetPosition(-1);
    // TODO: verify the motor was / was not set to the following values
  }

  SECTION("Should lerp motor to 4 by going 2->3->3.5")
  {
    joint.SetJointSpeed(4);
    CHECK(joint.GetSpeed() == 2);
    joint.SetJointSpeed(4);
    CHECK(joint.GetSpeed() == 3);
    joint.SetJointSpeed(4);
    CHECK(joint.GetSpeed() < 4);
    joint.SetJointSpeed(0);
    CHECK(joint.GetSpeed() > 0);
    CHECK(joint.GetSpeed() < 2);
  }

  SECTION("should boundary test the max and min angles for SetPostion")
  {
    joint.SetPosition(-10);
    // CHECK(joint.GetPosition() == minPosition);

    joint.SetPosition(200);
    // CHECK(joint.GetPosition() == maxPosition);
  }

  SECTION("should set zero offset to 25 degrees")
  {
    joint.SetZeroOffset(25);
    // TODO: verify the offset
  }

  SECTION("should return 5 for accelerometer data")
  {
    auto data = joint.GetAccelerometerData();
    CHECK(data.x == 5);
    CHECK(data.y == 5);
    CHECK(data.z == 5);
  }
}
}  // namespace sjsu