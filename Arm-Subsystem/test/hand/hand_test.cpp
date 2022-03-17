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

  Hand::MissionControlData hand_data;

  SECTION("1.1 Should return default angles at hand startup")
  {
    int default_angle = 0;

    CHECK_EQ(hand.GetThumbPosition(), default_angle);
    CHECK_EQ(hand.GetPointerPosition(), default_angle);
    CHECK_EQ(hand.GetMiddlePosition(), default_angle);
    CHECK_EQ(hand.GetRingPosition(), default_angle);
    CHECK_EQ(hand.GetPinkyPosition(), default_angle);
    CHECK_EQ(hand.GetCurrentHandMode(),
             Hand::MissionControlData::HandModes::kConcurrent);
    CHECK_EQ(hand.GetWristPitch(), default_angle);
    CHECK_EQ(hand.GetWristRoll(), default_angle);
  }

  SECTION("2.1 Should home the hand by setting the finger to max angles")
  {
    int max_angle = 180;

    hand.HomeHand(0, 0);
    CHECK_EQ(hand.GetThumbPosition(), max_angle);
    CHECK_EQ(hand.GetPointerPosition(), max_angle);
    CHECK_EQ(hand.GetMiddlePosition(), max_angle);
    CHECK_EQ(hand.GetRingPosition(), max_angle);
    CHECK_EQ(hand.GetPinkyPosition(), max_angle);
    // TODO - Add wrist homing tests
  }

  SECTION("3.1 Should correctly set the hand mode for every mode")
  {
    hand_data.hand_mode = Hand::MissionControlData::HandModes::kConcurrent;
    hand.HandleMovement(hand_data, 0);
    CHECK_EQ(hand.GetCurrentHandMode(),
             Hand::MissionControlData::HandModes::kConcurrent);

    hand_data.hand_mode = Hand::MissionControlData::HandModes::kPitch;
    hand.HandleMovement(hand_data, 0);
    CHECK_EQ(hand.GetCurrentHandMode(),
             Hand::MissionControlData::HandModes::kPitch);

    hand_data.hand_mode = Hand::MissionControlData::HandModes::kRoll;
    hand.HandleMovement(hand_data, 0);
    CHECK_EQ(hand.GetCurrentHandMode(),
             Hand::MissionControlData::HandModes::kRoll);

    hand_data.hand_mode = Hand::MissionControlData::HandModes::kTransport;
    hand.HandleMovement(hand_data, 0);
    CHECK_EQ(hand.GetCurrentHandMode(),
             Hand::MissionControlData::HandModes::kTransport);
  }

  SECTION("3.2 Should set hand to transport mode angles")
  {
    int transport_angle = 0;

    hand_data.hand_mode = Hand::MissionControlData::HandModes::kTransport;
    hand.HandleMovement(hand_data, 0);
    CHECK_EQ(hand.GetThumbPosition(), transport_angle);
    CHECK_EQ(hand.GetPointerPosition(), transport_angle);
    CHECK_EQ(hand.GetMiddlePosition(), transport_angle);
    CHECK_EQ(hand.GetRingPosition(), transport_angle);
    CHECK_EQ(hand.GetPinkyPosition(), transport_angle);
  }

  SECTION("3.3 Should boundary test the wrist for setting roll position") {}

  SECTION("3.4 Should boundary test the wrist for setting pitch position") {}

  SECTION("3.5.1 Should set the fingers to the maximum angle") {}

  SECTION("3.5.2 Should set the fingers to the minimum angle") {}

  SECTION("3.5.3 Should set the speed to the maximum") {}

  SECTION("3.5.4 Should set the speed to the minimum") {}

  SECTION("3.5.5 Should set the fingers to one beyond the maximum") {}

  SECTION("3.5.6 Should set the fingers to one beyond the minimum") {}

  SECTION("3.5.7 Should set the speed to one beyond the maximum") {}

  SECTION("3.5.8 Should set the speed to one beyond the minimum") {}
}

}  // namespace sjsu::arm