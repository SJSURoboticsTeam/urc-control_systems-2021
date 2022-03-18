#include "testing/testing_frameworks.hpp"
#include "devices/actuators/servo/rmd_x.hpp"

#include "arm_system.hpp"

namespace sjsu
{
TEST_CASE("Arm system testing")
{
  auto accel_value = units::acceleration::meters_per_second_squared_t(5);
  Accelerometer::Acceleration_t example_acceleration = { accel_value,
                                                         accel_value,
                                                         accel_value };

  arm::RoverArmSystem::GeneralMissionControlData mc_data;
  arm::Hand::MissionControlData hand_mc_data;
  arm::Arm::MissionControlData arm_mc_data;

  Mock<Pca9685> pca;

  Mock<Can> mock_can;
  Fake(Method(mock_can, Can::ModuleInitialize));
  Fake(OverloadedMethod(mock_can, Can::Send, void(const Can::Message_t &)));
  Fake(Method(mock_can, Can::Receive));
  Fake(Method(mock_can, Can::HasData));

  Mock<I2c> mock_i2c;
  Fake(Method(mock_i2c, I2c::ModuleInitialize));

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
  arm::Arm arm(rotunda, shoulder, elbow);

  arm::Finger pinky(0);
  arm::Finger ring(1);
  arm::Finger middle(2);
  arm::Finger pointer(3);
  arm::Finger thumb(4);
  arm::WristJoint wrist(motor, motor, spyed_mpu);
  arm::Hand hand(pca.get(), wrist, pinky, ring, middle, pointer, thumb);

  arm::RoverArmSystem arm_system(arm, hand);

  SECTION("1.1 should initialize and return default values")
  {
    int default_angle = 0;
    CHECK_EQ(hand.GetWristRoll(), default_angle);
    CHECK_EQ(hand.GetWristPitch(), default_angle);
    CHECK_EQ(hand.GetThumbPosition(), default_angle);
    CHECK_EQ(hand.GetPointerPosition(), default_angle);
    CHECK_EQ(hand.GetMiddlePosition(), default_angle);
    CHECK_EQ(hand.GetRingPosition(), default_angle);
    CHECK_EQ(hand.GetPinkyPosition(), default_angle);
    CHECK_EQ(hand.GetCurrentHandMode(),
             arm::Hand::MissionControlData::HandModes::kConcurrent);

    CHECK_EQ(arm.GetElbowPosition(), 0);
    CHECK_EQ(arm.GetElbowSpeed(), 0);
    CHECK_EQ(arm.GetRotundaPosition(), 0);
    CHECK_EQ(arm.GetRotundaOffsetAngle(), 0);
    CHECK_EQ(arm.GetRotundaSpeed(), 0);
    CHECK_EQ(arm.GetShoulderPosition(), 0);
    CHECK_EQ(arm.GetShoulderSpeed(), 0);
    CHECK_EQ(arm.GetCurrentArmMode(),
             arm::Arm::MissionControlData::ArmModes::kConcurrent);
  }

  SECTION("2.1 should return default GET parameters")
  {
    std::string expected_parameter =
        "?heartbeat_count=0&is_operational=0&arm_mode=C&hand_mode=C&arm_speed="
        "0&battery=90&rotunda_angle=0&shoulder_angle=0&elbow_angle=0&wrist_"
        "roll=0&wrist_pitch=0&pinky_angle=0&ring_angle=0&middle_angle=0&"
        "pointer_angle=0&thumb_angle=0";
    std::string actual_parameter =
        arm_system.CreateGETRequestParameterWithRoverStatus();
    CHECK_EQ(expected_parameter, actual_parameter);
  }

  SECTION("3.1 should parse json response correctly")
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
    arm_system.ParseMissionControlCommands(example_response);
    mc_data      = arm_system.GetMCData();
    arm_mc_data  = arm_system.GetArmMCData();
    hand_mc_data = arm_system.GetHandMCData();
    CHECK_EQ(mc_data.heartbeat_count, 0);
    CHECK_EQ(mc_data.is_operational, 1);
    CHECK_EQ(arm_mc_data.arm_mode, arm::Arm::MissionControlData::ArmModes('A'));
    CHECK_EQ(hand_mc_data.hand_mode,
             arm::Hand::MissionControlData::HandModes('H'));
    CHECK_EQ(mc_data.speed, 5);
    CHECK_EQ(arm_mc_data.arm_angles.rotunda, 5);
    CHECK_EQ(arm_mc_data.arm_angles.shoulder, 5);
    CHECK_EQ(arm_mc_data.arm_angles.elbow, 5);
    CHECK_EQ(hand_mc_data.wrist_data.roll, 5);
    CHECK_EQ(hand_mc_data.wrist_data.pitch, 5);
    CHECK_EQ(hand_mc_data.finger_angles.pinky_angle, 5);
    CHECK_EQ(hand_mc_data.finger_angles.ring_angle, 5);
    CHECK_EQ(hand_mc_data.finger_angles.middle_angle, 5);
    CHECK_EQ(hand_mc_data.finger_angles.pointer_angle, 5);
    CHECK_EQ(hand_mc_data.finger_angles.thumb_angle, 5);
  }
}
}  // namespace sjsu