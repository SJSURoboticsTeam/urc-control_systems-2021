#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "rover_arm_system.hpp"
#include "joint.hpp"

namespace sjsu
{
TEST_CASE("Drive system testing")
{
  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  Mock<I2c> mock_i2c;
  Fake(Method(mock_i2c, I2c::ModuleInitialize));

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);
  RmdX motor(network, 0x142);
  Mpu6050 mpu(mock_i2c.get(), 0x69);
  Mock<Mpu6050> mock_mpu(mpu);
  Fake(Method(mock_mpu, Mpu6050::ModuleInitialize));
  Mpu6050 & test_mpu = mock_mpu.get();

  arm::Joint joint(motor, test_mpu);

  std::string example_response;

  SECTION("checking default values")
  {
    joint.Initialize();
    joint.speed_ == 0_rpm;
  }
}
}  // namespace sjsu