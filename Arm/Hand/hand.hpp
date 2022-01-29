#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
namespace sjsu::arm
{
class Hand
{
 public:
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
  void HandleHandMovement(float speed,
                          float thumb_position,
                          float pointer_position,
                          float middle_position,
                          float ring_position,
                          float pinky_position)
  {
    thumb_.SetSpeed(speed);
    thumb_.SetPosition(thumb_position);
    pointer_.SetSpeed(speed);
    pointer_.SetPosition(pointer_position);
    middle_.SetSpeed(speed);
    middle_.SetPosition(middle_position);
    ring_.SetSpeed(speed);
    ring_.SetPosition(ring_position);
    pinky_.SetSpeed(speed);
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

  void CloseHand(float speed)
  {
    HandleHandMovement(speed, thumb_.GetMaxAngle(), pointer_.GetMaxAngle(),
                       middle_.GetMaxAngle(), ring_.GetMaxAngle(),
                       pinky_.GetMaxAngle());
  }

  void OpenHand(float speed)
  {
    HandleHandMovement(speed, thumb_.GetMinAngle(), pointer_.GetMinAngle(),
                       middle_.GetMinAngle(), ring_.GetMinAngle(),
                       pinky_.GetMinAngle());
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