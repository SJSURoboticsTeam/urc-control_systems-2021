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

  StaticMemoryResource<1024> memory_resource;
  CanNetwork network(mock_can.get(), &memory_resource);

  sjsu::RmdX rotunda_motor(network, 0x141);
  sjsu::RmdX shoulder_motor(network, 0x142);
  sjsu::RmdX elbow_motor(network, 0x143);
  sjsu::RmdX left_wrist_motor(network, 0x144);
  sjsu::RmdX right_wrist_motor(network, 0x145);

  sjsu::Mpu6050 rotunda_mpu(mock_i2c.get(), 0x68);
  sjsu::Mpu6050 shoulder_mpu(mock_i2c.get(), 0x69);
  sjsu::Mpu6050 elbow_mpu(mock_i2c.get(), 0x6A);
  sjsu::Mpu6050 wrist_mpu(mock_i2c.get(), 0x6B);

  Mock<Mpu6050> mock_rotunda_mpu(rotunda_mpu);
  Mock<Mpu6050> mock_shoulder_mpu(shoulder_mpu);
  Mock<Mpu6050> mock_elbow_mpu(elbow_mpu);
  Mock<Mpu6050> mock_wrist_mpu(wrist_mpu);

  Fake(Method(mock_rotunda_mpu, Mpu6050::ModuleInitialize));
  Fake(Method(mock_shoulder_mpu, Mpu6050::ModuleInitialize));
  Fake(Method(mock_elbow_mpu, Mpu6050::ModuleInitialize));
  Fake(Method(mock_wrist_mpu, Mpu6050::ModuleInitialize));

  When(Method(mock_rotunda_mpu, Mpu6050::Read))
      .AlwaysReturn(example_acceleration);
  When(Method(mock_shoulder_mpu, Mpu6050::Read))
      .AlwaysReturn(example_acceleration);
  When(Method(mock_elbow_mpu, Mpu6050::Read))
      .AlwaysReturn(example_acceleration);
  When(Method(mock_wrist_mpu, Mpu6050::Read))
      .AlwaysReturn(example_acceleration);

  Mpu6050 & spyed_rotunda_mpu  = mock_rotunda_mpu.get();
  Mpu6050 & spyed_shoulder_mpu = mock_shoulder_mpu.get();
  Mpu6050 & spyed_elbow_mpu    = mock_elbow_mpu.get();
  Mpu6050 & spyed_wrist_mpu    = mock_wrist_mpu.get();

  sjsu::arm::Joint shoulder(shoulder_motor, spyed_shoulder_mpu);
  sjsu::arm::Joint elbow(elbow_motor, spyed_elbow_mpu);
  sjsu::arm::WristJoint wrist(left_wrist_motor, right_wrist_motor,
                              spyed_wrist_mpu);
  sjsu::arm::Joint rotunda(rotunda_motor, spyed_rotunda_mpu, 0_deg, 3600_deg,
                           1800_deg);

  sjsu::arm::RoverArmSystem arm(rotunda, shoulder, elbow, wrist);

  SECTION("should initialize and return default values")
  {
    // arm.Initialize();
  }

  SECTION("should return default GET parameters")
  {
    std::string expected_parameter =
        "?heartbeat_count=0&is_operational=0&rotunda_speed=0&rotunda_angle=0&"
        "shoulder_speed=0&shoulder_angle=0&elbow_angle=0&elbow_speed=0&wrist_"
        "speed=0&wrist_roll=0&wrist_pitch=0&finger.pinky=0&finger.ring=0&"
        "finger.middle=0&finger.pointer=0&finger.thumb=0&modes='D'";
    CHECK(expected_parameter == arm.GETParameters());
  }

  SECTION("should parse json response correctly")
  {
    // CHECK(arm.ParseJSONResponse() == "Fill");
  }
}
}  // namespace sjsu