#pragma once
#include "peripherals/uart.hpp"
namespace sjsu::arm
{
class Hand
{
  /// The hand has its own MCU that communicates with the arm via UART.

 public:
  Hand(Uart & uart) : uart_(uart) {}
  void Initialize() {}
  void Enable(bool enable = true) {}

 private:
  Uart & uart_;
};
}  // namespace sjsu::arm
