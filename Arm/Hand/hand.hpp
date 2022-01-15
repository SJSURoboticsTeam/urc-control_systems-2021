#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
namespace sjsu::arm
{
class Hand
{
  /// The hand has its own MCU that communicates with the arm via UART.
 public:
  // Hand(Uart & uart) : uart_(uart) {}
  void Initialize(){};

  struct Fingers
  {
    float pinky_angle   = 0;
    float ring_angle    = 0;
    float middle_angle  = 0;
    float pointer_angle = 0;
    float thumb_angle   = 0;
  };
  struct accelerations
  {
    float x;
    float y;
    float z;
  };

  Hand(sjsu::arm::WristJoint wrist) : wrist_(wrist) {}

  void HomeWrist(float rotunda_offset)
  {
    wrist_.GetAccelerometerData();
    HomePitch(rotunda_offset);
    HomeRoll();
  };

  // TODO: find a way to set the ZeroOffsets
  void HomePitch(float rotunda_offset)
  {
    float wrist_offset =
        float(atan(wrist_.acceleration.y / wrist_.acceleration.z)) +
        rotunda_offset;
    wrist_.SetPitchPosition(wrist_offset);
  }
  //Get finger position
void SetThumbPosition()
{
  
};

void SetMiddlePosition()
{
  
};

void SetPinkyPosition(float target_pinky_angle)
{
  fingers_.pinky_angle = std:clamp(target_pinky_angle, min_angle, max_angle)
};

void SetPointerPosition(float target_pointer_angle)
{
  fingers_.pointer_angle = std:clamp(target_pointer_angle, min_angle, max_angle)
};

void SetRingPosition(float target_ring_position)
{
  fingers_.ring_angle = std:clamp(target_ring_angle, min_angle, max_angle)
};
  // can't home yet
  void HomeRoll(){};

  int GetThumbPosition()
  {
    return fingers_.thumb_angle;
  };

  int GetMiddlePosition()
  {
    return fingers_.middle_angle;
  };

  int GetPinkyPosition()
  {
    return fingers_.pinky_angle;
  };

  int GetPointerPosition()
  {
    return fingers_.pointer_angle;
  };

  int GetRingPosition()
  {
    return fingers_.ring_angle;
  };
  sjsu::arm::WristJoint wrist_;



 private:
 int max_angle = 180;
 int min_angle = 0;
 Fingers fingers_;
  // Uart & uart_;
};

}  // namespace sjsu::arm