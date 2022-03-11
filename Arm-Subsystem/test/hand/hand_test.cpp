#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/servo.hpp"
#include "Hand/finger.hpp"
#include "Hand/hand.hpp"

namespace sjsu::arm
{
TEST_CASE("Hand Testing Case...")
{
  Mock<Pca9685> pca;
  Finger pinky_finger(0);
  Finger ring_finger(1);
  Finger middle_finger(2);
  Finger pointer_finger(3);
  Finger thumb_finger(4);

  auto accel_value = units::acceleration::meters_per_second_squared_t(5);
  sjsu::Accelerometer::Acceleration_t example_acceleration = { accel_value,
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

  WristJoint wrist(left_motor, right_motor, spyed_mpu);

  Hand hand(pca.get(), wrist, pinky_finger, ring_finger, middle_finger,
            pointer_finger, thumb_finger);

  SECTION() {}
}

}  // namespace sjsu::arm