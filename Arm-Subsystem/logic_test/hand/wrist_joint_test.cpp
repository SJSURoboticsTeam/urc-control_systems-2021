#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

#include "arm_system.hpp"
#include "Hand/wrist_joint.hpp"
#include "joint.hpp"

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

  SECTION("1.1 should return zero angle at rover startup")
  {
    wrist_joint.Initialize();
    CHECK_EQ(wrist_joint.GetPitchPosition(), 0);
  }

  SECTION("1.2 should return non zero angle after moving wrist once ")
  {
    wrist_joint.SetPitchPosition(30, 30);
    CHECK(wrist_joint.GetPitchPosition() != 0);
  }

  SECTION("2.1 should return zero angle at rover startup")
  {
    wrist_joint.Initialize();
    CHECK_EQ(wrist_joint.GetRollPosition(), 0);
  }

  SECTION("2.2 should return non zero angle after moving wrist once ")
  {
    wrist_joint.SetRollPosition(30, 30);
    CHECK(wrist_joint.GetRollPosition() != 0);
  }

  SECTION("3.1 should return zero when setting pitch to zero")
  {
    wrist_joint.SetPitchPosition(0, 30);
    CHECK(wrist_joint.GetPitchPosition() == 0);
  }

  SECTION("3.2 Should return 90 when setting pitch to 90")
  {
    wrist_joint.SetPitchPosition(90, 30);
    CHECK(wrist_joint.GetPitchPosition() == 90);
  }

  SECTION("3.3 Should return 180 when setting pitch to 180")
  {
    wrist_joint.SetPitchPosition(180, 30);
    CHECK(wrist_joint.GetPitchPosition() == 180);
  }

  SECTION("3.4 Should return 0 when setting pitch to -1")
  {
    wrist_joint.SetPitchPosition(-1, 30);
    CHECK(wrist_joint.GetPitchPosition() == 0);
  }

  SECTION("3.5 Should return 180 when setting pitch to 181")
  {
    wrist_joint.SetPitchPosition(181, 30);
    CHECK(wrist_joint.GetPitchPosition() == 180);
  }

  SECTION("4.1 should return zero when setting roll to zero")
  {
    wrist_joint.SetRollPosition(0, 30);
    CHECK(wrist_joint.GetRollPosition() == 0);
  }

  SECTION("4.2 Should return 90 when setting roll to 90")
  {
    wrist_joint.SetRollPosition(90, 30);
    CHECK(wrist_joint.GetRollPosition() == 90);
  }

  SECTION("4.3 Should return 180 when setting roll to 180")
  {
    wrist_joint.SetRollPosition(180, 30);
    CHECK(wrist_joint.GetRollPosition() == 180);
  }

  SECTION("4.4 Should return 0 when setting roll to -1")
  {
    wrist_joint.SetRollPosition(-1, 30);
    CHECK(wrist_joint.GetRollPosition() == 0);
  }

  SECTION("4.5 Should return 180 when setting roll to 181")
  {
    wrist_joint.SetRollPosition(181, 30);
    CHECK(wrist_joint.GetRollPosition() == 180);
  }
}
}  // namespace sjsu