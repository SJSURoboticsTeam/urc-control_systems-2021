#pragma once

#include "utility/status.hpp"

namespace sjsu
{
/// Abstract interface for devices that drive basic rotational motors.
/// including motors that that rotate continually such as brushed and brushless
/// DC/AC motors and excluding stepper and servo motors.
class Motor
{
 public:
  /// Initialize peripherals required to communicate with motor controller. Must
  /// be the first method called on this driver. After this is called,
  /// `Enable()` can be called.
  virtual Returns<void> Initilize() = 0;

  /// This method must be called before running `Speed()`. This function will
  /// configure the motor controller settings as defined through the constructor
  /// of this interface's implementation. Some implementations have more detail
  /// or settings than others.
  virtual Returns<void> Enable() = 0;

  /// This method makes the motor turn at a given percentage of its maximum
  /// speed in the direction dictated by the sign of the percent. percentage
  /// ranges from -1.0f to 1.0f and will clamp the values if greater than this
  /// range.
  virtual Returns<void> Speed(float percentage) = 0;
};
}  // namespace sjsu
