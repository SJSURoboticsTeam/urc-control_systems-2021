#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
namespace sjsu::arm
{
class Hand
{
  /// The hand has its own MCU that communicates with the arm via UART.
 public:
  // Hand(Uart & uart) : uart_(uart) {}

  struct accelerations
  {
    float x;
    float y;
    float z;
  };

  Hand(sjsu::arm::WristJoint & wrist,
       sjsu::arm::Finger & pinky,
       sjsu::arm::Finger & ring,
       sjsu::arm::Finger & middle,
       sjsu::arm::Finger & pointer,
       sjsu::arm::Finger & thumb)
      : wrist_(wrist),
        pinky_(pinky),
        ring_(ring),
        middle_(middle),
        pointer_(pointer),
        thumb_(thumb)
  {
  }

  void Initialize()
  {
    wrist_.Initialize();
    pinky_.Initialize();
    ring_.Initialize();
    middle_.Initialize();
    pointer_.Initialize();
    thumb_.Initialize();
  }
  void HandleHandMovement(float thumb_position,
                          float pointer_position,
                          float middle_position,
                          float ring_position,
                          float pinky_position)
  {
    thumb_.SetPosition(thumb_position);
    pointer_.SetPosition(pointer_position);
    middle_.SetPosition(middle_position);
    ring_.SetPosition(ring_position);
    pinky_.SetPosition(pinky_position);
  }

  int GetThumbPosition()
  {
    return thumb_.GetPosition();
  };

  int GetPointerPosition()
  {
    return pointer_.GetPosition();
  };

  int GetMiddlePosition()
  {
    return middle_.GetPosition();
  };

  int GetRingPosition()
  {
    return ring_.GetPosition();
  };

  int GetPinkyPosition()
  {
    return pinky_.GetPosition();
  };

  void HomeWrist(float rotunda_offset)
  {
    wrist_.GetAccelerometerData();
    HomePitch(rotunda_offset);
    HomeRoll();
  };

  void HomePitch(float rotunda_offset)
  {
    float wrist_pitch_offset =
        float(atan(wrist_.acceleration_.y / wrist_.acceleration_.z)) +
        rotunda_offset;
    wrist_.SetPitchPosition(wrist_pitch_offset);
    wrist_.SetZeroPitchOffsets(wrist_pitch_offset);
  }

  // can't home yet
  void HomeRoll(){};

 private:
  // Uart & uart_;
  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Finger & pinky_;
  sjsu::arm::Finger & ring_;
  sjsu::arm::Finger & middle_;
  sjsu::arm::Finger & pointer_;
  sjsu::arm::Finger & thumb_;
};

}  // namespace sjsu::arm