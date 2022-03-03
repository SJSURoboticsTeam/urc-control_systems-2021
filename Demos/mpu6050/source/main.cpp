#include <cstdint>

#include "peripherals/lpc40xx/i2c.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"
#include "utility/log.hpp"

int main()
{
  sjsu::LogInfo("Mpu6050 Application Starting...");

  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();

  sjsu::Mpu6050 sensor(i2c, 0x66);  // increment 66-69 testing each 1 by 1

  // sjsu::Mpu6050 rotunda_sensor(i2c, 0x66);   // rotunda
  // sjsu::Mpu6050 shoulder_sensor(i2c, 0x67);  // shoulder
  // sjsu::Mpu6050 elbow_sensor(i2c, 0x68);     // elbow
  // sjsu::Mpu6050 wrist_sensor(i2c, 0x69);     // wrist

  sjsu::LogInfo("Initializing accelerometer peripherals...");
  sensor.Initialize();

  // rotunda_sensor.Initialize();
  // shoulder_sensor.Initialize();
  // elbow_sensor.Initialize();
  // wrist_sensor.Initialize();

  while (true)
  {
    auto current_acceleration = sensor.Read();
    current_acceleration.Print();

    // auto current_acceleration = rotunda_sensor.Read();
    // current_acceleration.Print();
    // current_acceleration = shoulder_sensor.Read();
    // current_acceleration.Print();
    // current_acceleration = elbow_sensor.Read();
    // current_acceleration.Print();
    // current_acceleration = wrist_sensor.Read();
    // current_acceleration.Print();

    sjsu::Delay(100ms);
  }
  return 0;
}
