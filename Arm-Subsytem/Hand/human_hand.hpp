#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
#include "Interface/hand.hpp"

namespace sjsu::arm
{
class Hand : public HandInterface
{
 public:
  struct MissionControlData : HandInterface::MissionControl
  {
    enum class HandModes : char
    {
      kPitch      = 'P',
      kRoll       = 'R',
      kConcurrent = 'C',
      kTransport  = 'T'
    };
    HandModes hand_mode = HandModes::kConcurrent;

    struct Finger
    {
      int pinky_angle   = 0;
      int ring_angle    = 0;
      int middle_angle  = 0;
      int pointer_angle = 0;
      int thumb_angle   = 0;
    };
    Finger fingers;

    struct Wrist
    {
      int roll  = 0;
      int pitch = 0;
    };
    Wrist wrist_data;
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

  void PrintHandData()
  {
    printf("Hand Finger Positions:\n");
    printf("Pinky Angle: %d\n", pinky_.GetPosition());
    printf("Ring Angle: %d\n", ring_.GetPosition());
    printf("Middle Angle: %d\n", middle_.GetPosition());
    printf("Pointer Angle: %d\n", pointer_.GetPosition());
    printf("Thumb Angle: %d\n", thumb_.GetPosition());

    wrist_.PrintWristData();
  }

  void HandleConcurrentMovement(MissionControlData::Finger finger_data,
                                MissionControlData::Wrist wrist_data,
                                float speed)
  {
    thumb_.SetSpeed(speed);
    thumb_.SetPosition(finger_data.thumb_angle);
    pointer_.SetSpeed(speed);
    pointer_.SetPosition(finger_data.pointer_angle);
    middle_.SetSpeed(speed);
    middle_.SetPosition(finger_data.middle_angle);
    ring_.SetSpeed(speed);
    ring_.SetPosition(finger_data.ring_angle);
    pinky_.SetSpeed(speed);
    pinky_.SetPosition(finger_data.pinky_angle);
    wrist_.HandleWristMovement(speed, wrist_data.roll, wrist_data.pitch);
  }

  // The following two functions are here to allow the rover arm system to
  // directly control the roll and the pitch of the wrist this is mainly for
  // testing due to the different hand drive modes, when the drive modes are
  // removed this may be removed as well
  void SetWristRollPosition(float wrist_roll, float speed)
  {
    wrist_.SetRollPosition(wrist_roll, speed);
  }

  void SetWristPitchPosition(float wrist_pitch, float speed)
  {
    wrist_.SetRollPosition(wrist_pitch, speed);
  }

  void SetHandTransportPosition()
  {
    float transport_angles = 0;
    thumb_.SetPosition(transport_angles);
    pointer_.SetPosition(transport_angles);
    middle_.SetPosition(transport_angles);
    ring_.SetPosition(transport_angles);
    pinky_.SetPosition(transport_angles);
    wrist_.HandleWristMovement(transport_angles, transport_angles);
  }

  void HandleMovement(MissionControlData hand_data, float speed)
  {
    switch (current_hand_mode_)
    {
      case MissionControlData::HandModes::kConcurrent:
        HandleConcurrentMovement(hand_data.fingers, hand_data.wrist_data,
                                 speed);
        break;
      case MissionControlData::HandModes::kPitch:
        SetWristPitchPosition(hand_data.wrist_data.pitch, speed);
        break;
      case MissionControlData::HandModes::kRoll:
        SetWristRollPosition(hand_data.wrist_data.roll, speed);
        break;
      case MissionControlData::HandModes::kTransport: 
        SetHandTransportPosition();
        break;
    }
  }

  int GetWristPitch() const
  {
    return wrist_.GetPitchPosition();
  }

  int GetWristRoll() const
  {
    return wrist_.GetRollPosition();
  }

  int GetThumbPosition() const
  {
    return thumb_.GetPosition();
  };

  int GetPointerPosition() const
  {
    return pointer_.GetPosition();
  };

  int GetMiddlePosition() const
  {
    return middle_.GetPosition();
  };

  int GetRingPosition() const
  {
    return ring_.GetPosition();
  };

  int GetPinkyPosition() const
  {
    return pinky_.GetPosition();
  };

  MissionControlData::HandModes GetCurrentHandMode() const
  {
    return current_hand_mode_;
  }

  void SetCurrentHandMode(MissionControlData::HandModes new_mode)
  {
    current_hand_mode_ = new_mode;
  }

  void HomeHand(float rotunda_offset_angle, float speed)
  {
    pinky_.Home();
    ring_.Home();
    middle_.Home();
    pointer_.Home();
    thumb_.Home();
    wrist_.Home(float(rotunda_offset_angle), speed);
  }

 private:
  MissionControlData::HandModes current_hand_mode_ =
      MissionControlData::HandModes::kConcurrent;

  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Finger & pinky_;
  sjsu::arm::Finger & ring_;
  sjsu::arm::Finger & middle_;
  sjsu::arm::Finger & pointer_;
  sjsu::arm::Finger & thumb_;
};

}  // namespace sjsu::arm