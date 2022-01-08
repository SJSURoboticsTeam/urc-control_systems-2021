#pragma once
#include "peripherals/uart.hpp"
namespace sjsu::arm
{
class Hand
{
  /// The hand has its own MCU that communicates with the arm via UART.

 public:
  // Hand(Uart & uart) : uart_(uart) {}
  Hand(){};
  void Initialize() {};
  void Enable(bool enable = true) {};
int GetThumbPosition()
{
  return 0;
};

int GetMiddlePosition()
{
  return 0;
};

int GetPinkyPosition()
{
  return 0;
};

int GetPointerPosition()
{
  return 0;
};

int GetRingPosition()
{
  return 0;
};
 private:
  // Uart & uart_;

};



}  // namespace sjsu::arm