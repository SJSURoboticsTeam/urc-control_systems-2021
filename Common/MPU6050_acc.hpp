/**
 * Header file for Accelerometer MPU6050 functionality.
 *
 * @author SJSU Robotics - Control Systems
 * @version Spring 2020
 */

#pragma once

#include <cstdint>
#include "L1_Peripheral/i2c.hpp"
#include "L2_HAL/sensors/movement/accelerometer.hpp"

namespace sjsu
{
namespace robotics
{
class MPU6050_acc : public Accelerometer
{
 public:
  static constexpr uint16_t kDataOffset         = 1;
  static constexpr float kRadiansToDegree       = 180.0f / 3.14f;
  static constexpr uint8_t kWhoAmIExpectedValue = 0x68;
  static constexpr uint8_t kMsbShift            = 8;

  // in units of 9.8 m/s^2 or "g"
  static constexpr int kMaxAccelerationScale[4]         = { 2, 4, 8, 16 };
  static constexpr uint8_t kSetMaxAccelerationScale[17] = {
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03
  };

  enum RegisterMap : uint8_t
  {
    kX          = 0x3B,
    kY          = 0x3D,
    kZ          = 0x3F,
    kWhoAmI     = 0x75, //This address should contain 0x68 (the device address) if this is really an MPU6050 sensor
    kDataConfig = 0x1C  //This register is where The FullScaleRange is stored
  };

/*
* The LSB of the MPU6050's address is determined by the AD0 pin
* In order to have 2 MPU6050's on the same bus, one of their AD0 pins should be set high, and the address 0x69 explicitly set
*/
  explicit constexpr MPU6050_acc(const I2c & i2c, uint8_t address = 0x68)
      : i2c_(i2c), accelerometer_address_(address)
  {
  }

  /*
  * Initilize the MPU6050 Accelerometer 
  * Returns true if the sensor is initilized sucessfully
  */
  bool Initialize() override
  {
    i2c_.Initialize();
    i2c_.Write(accelerometer_address_, { 0x6B, 0x0 });
    uint8_t who_am_i_received_value;
    uint8_t identity_register = RegisterMap::kWhoAmI;
    i2c_.WriteThenRead(accelerometer_address_,
                       &identity_register,
                       sizeof(identity_register),
                       &who_am_i_received_value,
                       sizeof(who_am_i_received_value));
    return (who_am_i_received_value == kWhoAmIExpectedValue);
  }

  /*
  * Returns the accleration value for an axis given as a register
  * The value is returned as a int16 where the real world value in G's
  * can be calculated by: RESULT / (INT16_MAX/GetFullScaleRange());
  */
  int16_t GetAxisValue(uint8_t register_number) const
  {
    int acc_reading;
    int16_t axis_acc;
    uint8_t acc_val[2];
    i2c_.WriteThenRead(accelerometer_address_,
                       { register_number },
                       acc_val,
                       sizeof(acc_val));
    acc_reading = (acc_val[0] << kMsbShift) | acc_val[1];
    axis_acc    = static_cast<int16_t>(acc_reading);
    return static_cast<int16_t>(axis_acc / kDataOffset);
  }

  /*
  * Functions to easily get each acceleration value for a given axis
  */
  int16_t X() const override
  {
    return GetAxisValue(RegisterMap::kX);
  }
  int16_t Y() const override
  {
    return GetAxisValue(RegisterMap::kY);
  }
  int16_t Z() const override
  {
    return GetAxisValue(RegisterMap::kZ);
  }

  /*
  * Pitch and roll recieve the acceleration values from the sensor and use them to 
  * return the angle of Pitch/Roll in degrees
  */
  float Pitch() const override
  {
    float x                 = static_cast<float>(X());
    float y                 = static_cast<float>(Y());
    float z                 = static_cast<float>(Z());
    float pitch_numerator   = x * -1.0f;
    float pitch_denominator = sqrtf((y * y) + (z * z));
    float pitch = atan2f(pitch_numerator, pitch_denominator) * kRadiansToDegree;
    return pitch;
  }
  float Roll() const override
  {
    float y = static_cast<float>(Y());
    float z = static_cast<float>(Z());
    return (atan2f(y, z) * kRadiansToDegree);
  }

  /*
  * GetFullScaleRange will retrive the scaling factor used by the sensor when a value is read
  * Knowing the scale is required to convert into G's (m/s^2)
  * 
  * SetFullScaleRange will modify the scaling factor used inorder that higher max G's or greater accuracy may be used
  * Valid FullScaleRanges(+- G's) for the MPU6050 are 2,4,8,& 16
  */ 
  int GetFullScaleRange() const override
  {
    uint8_t config_reg = RegisterMap::kDataConfig;
    uint8_t full_scale_value;
    i2c_.WriteThenRead(accelerometer_address_,
                       { config_reg },
                       &full_scale_value,
                       sizeof(full_scale_value));
    full_scale_value = (full_scale_value & 0x18) >> 3;
    int range        = kMaxAccelerationScale[full_scale_value];
    return range;
  }
  void SetFullScaleRange(uint8_t range_value) override
  {
    range_value &= 0x1F;
    uint8_t config_reg = RegisterMap::kDataConfig;
    uint8_t send_range = kSetMaxAccelerationScale[range_value] << 3;
    uint8_t full_scale_write_buffer[2] = { config_reg, send_range };
    i2c_.Write(accelerometer_address_,
               full_scale_write_buffer,
               sizeof(full_scale_write_buffer));
  }

 private:
  const I2c & i2c_;
  uint8_t accelerometer_address_;
};

}  // namespace robotics
}  // namespace sjsu
