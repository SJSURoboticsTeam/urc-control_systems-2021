/**
 * Header file for magnetic encoder AS5601 functionality.
 *
 * @author SJSU Robotics - Control Systems
 * @version Spring 2020
 */

#pragma once

#include <cstdint>

#include "utility/log.hpp"
#include "L1_Peripheral/i2c.hpp"
#include "L1_Peripheral/lpc40xx/i2c.hpp"
#include "utility/map.hpp"
#include "third_party/units/units.h"

namespace sjsu
{
namespace robotics
{
class MagneticEncoder
{
 public:
  static constexpr uint8_t kMagneticEncoderAddress   = 0x36;
  static constexpr uint8_t kMagneticEncoderMagnitude = 0x1B;
  static constexpr uint8_t kEncoderAngle             = 0x0E;
  static constexpr units::angle::degree_t kZeroAngle = 0_deg;

  /**
   * Constructor for MagneticEncoder object. Take i2c as parameter.
   */
  explicit constexpr MagneticEncoder(sjsu::I2c & i2c)
      : current_angle_(kZeroAngle), i2c_(i2c)
  {
  }

  /**
   * Initializes MagneticEncoder object.
   */
  void Initialize()
  {
    i2c_.Initialize();
  }

  /**
   * Gets the current angle of MagneticEncoder relative to steering motor.
   *
   * @return The angle of the magetic encoder.
   */
  units::angle::degree_t GetAngle() const
  {
    uint8_t register_vals[2];
    uint16_t angle;
    uint16_t mappedAngle;

    i2c_.WriteThenRead(kMagneticEncoderAddress,
                       { kEncoderAngle },
                       register_vals,
                       sizeof(register_vals));

    angle       = register_vals[0] << 8 | register_vals[1];
    mappedAngle = sjsu::Map(angle, 63, 3903, 0, 360);
    units::angle::degree_t convertedMappedAngle =
        units::make_unit<units::angle::degree_t>(mappedAngle);
    LOG_INFO("\nAngle: %d\nIntMapped: %d\nUnitMapped: %d\n",
             angle,
             mappedAngle,
             convertedMappedAngle.to<int>());
    return convertedMappedAngle;
  }

  /**
   * Zeros out the magentic encoder current angle.
   */
  void SetZero()
  {
    current_angle_ = kZeroAngle;
  }

 private:
  units::angle::degree_t current_angle_;
  const sjsu::I2c & i2c_;
};
}  // namespace robotics
}  // namespace sjsu
