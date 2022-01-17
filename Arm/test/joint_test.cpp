#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "rover_arm_system.hpp"
#include "joint.hpp"

namespace sjsu
{
TEST_CASE("Joint system testing")
{
  auto accel_value = units::acceleration::meters_per_second_squared_t(0);
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

  SECTION("1.1 - 3.1 should initialize and return default values")
  {
    joint.Initialize();
    CHECK(joint.GetSpeed() == 0);
    CHECK(joint.GetPosition() == 0);
  }

  SECTION("4.1  Mock MPU return 0 for X,Y, Z")
  {
    auto data = joint.GetAccelerometerData();
    CHECK_EQ(data.x,0.0);
    CHECK_EQ(data.y,0.0);
    CHECK_EQ(data.z,0.0);
  }

  SECTION("4.2  Mock MPU return 90 for X,Y, Z")
  {
    accel_value = units::acceleration::meters_per_second_squared_t(90);
    example_acceleration = {accel_value,
                            accel_value,
                            accel_value };

  When(Method(mock_mpu, Mpu6050::Read)).AlwaysReturn(example_acceleration);

    auto data = joint.GetAccelerometerData();
    CHECK_EQ(data.x,90.0);
    CHECK_EQ(data.y,90.0);
    CHECK_EQ(data.z,90.0);
    
    
  }

  SECTION("5.1 Should return 0 when joint speed is set to 0")
  {
    joint.SetJointSpeed(0);
    CHECK_EQ(joint.GetSpeed() , 0);
  }

  SECTION("5.2 Should return 100 when joint speed is set to 101")
  {
    joint.SetJointSpeed(100);
    CHECK_EQ(joint.GetSpeed(), 100);
  }

  SECTION("5.3 Should return 101 when joint speed is set to 100")
  {
    joint.SetJointSpeed(101);
    CHECK_EQ(joint.GetSpeed(), 100);
  }
  SECTION("5.4 Should return -100 when joint speed is set to -100")
  {
    joint.SetJointSpeed(-100);
    CHECK_EQ(joint.GetSpeed(), -100);
  }

  SECTION("5.5 Should return -100 when joint speed is set to -101")
  {
    joint.SetJointSpeed(-101);
    CHECK_EQ(joint.GetSpeed(), -100);
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

  // SECTION("Should lerp motor to 4 by going 2->3->3.5")
  // {
  //   joint.SetJointSpeed(4);
  //   CHECK(joint.GetSpeed() == 2);
  //   joint.SetJointSpeed(4);
  //   CHECK(joint.GetSpeed() == 3);
  //   joint.SetJointSpeed(4);
  //   CHECK(joint.GetSpeed() < 4);
  //   joint.SetJointSpeed(0);
  //   CHECK(joint.GetSpeed() > 0);
  //   CHECK(joint.GetSpeed() < 2);
  // }

  SECTION("6.1 Should return 0 when joint position is set to 0")
  {
    joint.SetPosition(0);
    CHECK_EQ(joint.GetPosition(), 0);
  }

   SECTION("6.2 Should return 0 when joint position is set to -1")
  {
    joint.SetPosition(-1);
    CHECK_EQ(joint.GetPosition(), 0);
  }

  SECTION("6.3 Should return 180 when joint position is set to 180")
  {
    joint.SetPosition(180);
    CHECK_EQ(joint.GetPosition(), 180);
  }

  SECTION("6.4 Should return 180 when joint position is set to 181")
  {
    joint.SetPosition(181);
    CHECK_EQ(joint.GetPosition(), 180);
  }


  SECTION("7.1 Sets offset to 0 when MPU is flat")
  {

        accel_value = units::acceleration::meters_per_second_squared_t(90);
    example_acceleration = {units::acceleration::meters_per_second_squared_t(0),
                            units::acceleration::meters_per_second_squared_t(0),
                            units::acceleration::meters_per_second_squared_t(9.81) };

  When(Method(mock_mpu, Mpu6050::Read)).AlwaysReturn(example_acceleration);
  if()
    joint.SetZeroOffset(25);
  }

  SECTION("7.2 Sets offset to 90 when MPU is vertical")
  {
    
    joint.SetZeroOffset(25);
    // TODO: verify the offset
  }


}
}  // namespace sjsu