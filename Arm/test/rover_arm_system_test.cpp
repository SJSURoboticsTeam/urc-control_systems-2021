#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "rover_arm_system.hpp"

namespace sjsu
{
TEST_CASE("Arm system testing")
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

  Mock<I2c> mock_i2c;
  Fake(Method(mock_i2c, I2c::ModuleInitialize));

  Mock<Servo> mock_servo;

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  RmdX motor(network, 0x141);

  Mpu6050 mpu(mock_i2c.get(), 0x68);

  Mock<Mpu6050> mock_mpu(mpu);

  Fake(Method(mock_mpu, Mpu6050::ModuleInitialize));

  When(Method(mock_mpu, Mpu6050::Read)).AlwaysReturn(example_acceleration);

  Mpu6050 & spyed_mpu = mock_mpu.get();

  arm::ArmJoint shoulder(motor, spyed_mpu);
  arm::ArmJoint elbow(motor, spyed_mpu);
  arm::ArmJoint rotunda(motor, spyed_mpu, 0, 3600, 1800);

  arm::Finger pinky(mock_servo.get());
  arm::Finger ring(mock_servo.get());
  arm::Finger middle(mock_servo.get());
  arm::Finger pointer(mock_servo.get());
  arm::Finger thumb(mock_servo.get());
  arm::WristJoint wrist(motor, motor, spyed_mpu);
  arm::Hand hand(wrist, pinky, ring, middle, pointer, thumb);

  arm::RoverArmSystem arm(rotunda, shoulder, elbow, hand);

  SECTION("should initialize and return default values")
  {
    // arm.Initialize();
    CHECK_EQ(hand.GetWristRoll(), 0);
    CHECK_EQ(hand.GetWristPitch(), 0);
  }

  SECTION("should return default GET parameters")
  {
    std::string expected_parameter =
        "?heartbeat_count=0&is_operational=0&arm_mode=C&hand_mode=C&arm_speed=0&"
        "battery=90&rotunda_angle=0&shoulder_angle=0&elbow_"
        "angle=0&wrist_roll=0&wrist_pitch=0&pinky_angle=0&"
        "ring_angle=0&middle_angle=0&pointer_angle=0&thumb_"
        "angle=0";
    std::string actual_parameter = arm.GETParameters();
    CHECK(expected_parameter == actual_parameter);
  }

  SECTION("should parse json response correctly")
  {
    std::string example_response =
        "\r\n\r\n{\n"
        "  \"heartbeat_count\": 0,\n"
        "  \"is_operational\": 1,\n"
        "  \"arm_mode\": \"A\",\n"
        "  \"hand_mode\": \"H\",\n"
        "  \"arm_speed\": 5,\n"
        "  \"rotunda_angle\": 5,\n"
        "  \"shoulder_angle\": 5,\n"
        "  \"elbow_angle\": 5,\n"
        "  \"wrist_roll\": 5,\n"
        "  \"wrist_pitch\": 5,\n"
        "  \"pinky_angle\": 5,\n"
        "  \"ring_angle\": 5,\n"
        "  \"middle_angle\": 5,\n"
        "  \"pointer_angle\": 5,\n"
        "  \"thumb_angle\": 5\n"
        "}";
    arm.ParseJSONResponse(example_response);
    CHECK(arm.mc_data_.heartbeat_count == 0);
    CHECK(arm.mc_data_.is_operational == 1);
  }
}
}  // namespace sjsu