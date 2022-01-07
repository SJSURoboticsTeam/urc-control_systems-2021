#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
// #include "arm_helper.hpp"
#include "joint.hpp"

namespace sjsu
{
TEST_CASE("Drive system testing")
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
    CHECK(joint.speed_ == 0_rpm);
  }

  SECTION("should boundary test setting the motor angle")
  {
    joint.SetPosition(5_deg);
    joint.SetPosition(180_deg);
    joint.SetPosition(181_deg);
    joint.SetPosition(0_deg);
    joint.SetPosition(-1_deg);
    // TODO: verify the motor was / was not set to the following values
  }

  SECTION("should set zero offset to 25 degrees")
  {
    joint.SetZeroOffset(25_deg);
    // TODO: verify the offset
  }

  SECTION("should return 5 for accelerometer data")
  {
    auto data = joint.GetAccelerometerData();
    CHECK(data.x == accel_value);
    CHECK(data.y == accel_value);
    CHECK(data.z == accel_value);
  }
}
}  // namespace sjsu