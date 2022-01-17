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
  //Hand(Uart & uart) : uart_(uart) {}

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
       sjsu::arm::Finger & thumb
      ) : wrist_(wrist), pinky_(pinky), ring_(ring), middle_(middle), pointer_(pointer), thumb_(thumb) {}

  void Initialize()
  {
    wrist_.Initialize();
    pinky_.Initialize();
    ring_.Initialize();
    middle_.Initialize();
    pointer_.Initialize();
    thumb_.Initialize();
  }

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

  // can't home yet
  void HomeRoll(){};

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
  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Finger & pinky_;
  sjsu::arm::Finger & ring_;
  sjsu::arm::Finger & middle_;
  sjsu::arm::Finger & pointer_;
  sjsu::arm::Finger & thumb_;
  private:
  // Uart & uart_;
};

}  // namespace sjsu::arm