#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
#include "Hand/wrist_joint.hpp"

namespace sjsu
{
TEST_CASE("Wrist joint system testing")
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
  RmdX left_motor(network, 0x141);
  RmdX right_motor(network, 0x142);

  Mock<I2c> mock_i2c;
  Fake(Method(mock_i2c, I2c::ModuleInitialize));

  Mpu6050 mpu(mock_i2c.get(), 0x69);

  Mock<Mpu6050> mock_mpu(mpu);
  Fake(Method(mock_mpu, Mpu6050::ModuleInitialize));
  When(Method(mock_mpu, Mpu6050::Read)).AlwaysReturn(example_acceleration);

  Mpu6050 & spyed_mpu = mock_mpu.get();

  arm::WristJoint wrist_joint(left_motor, right_motor, spyed_mpu);

  SECTION("should return default values")
  {
    wrist_joint.Initialize();
    CHECK_EQ(wrist_joint.GetPitchPosition(), 0);
    CHECK_EQ(wrist_joint.GetRollPosition(), 0);
  }
}
}  // namespace sjsu