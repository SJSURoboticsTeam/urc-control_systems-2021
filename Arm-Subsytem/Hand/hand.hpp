#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
namespace sjsu::arm
{
class Hand
{
 public:
  struct MissionControlHandData
  {

    enum class HandModes : char
    {
      kPitch      = 'P',
      kRoll       = 'R',
      kClose      = 'F',
      kOpen       = 'O',
      kConcurrent = 'C'
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

    int wrist_roll  = 0;
    int wrist_pitch = 0;
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

  void HandleConcurrentMovement(float speed,
                          float thumb_position,
                          float pointer_position,
                          float middle_position,
                          float ring_position,
                          float pinky_position,
                          float roll_position,
                          float pitch_position)
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
    wrist_.HandleWristMovement(speed, roll_position, pitch_position);
  }

  // The following two functions are here to allow the rover arm system to
  // directly control the roll and the pitch of the wrist this is mainly for
  // testing due to the different hand drive modes, when the drive modes are
  // removed this may be removed as well
  void SetWristRollPosition(float speed, float roll_position)
  {
    wrist_.SetRollPosition(speed, roll_position);
  }

  void SetWristPitchPosition(float speed, float pitch_position)
  {
    wrist_.SetRollPosition(speed, pitch_position);
  }

  void HandleMovement(float speed,
                       float thumb_angle,
                       float pointer_angle,
                       float middle_angle,
                       float ring_angle,
                       float pinky_angle,
                       float roll,
                       float pitch)
  {
    switch (current_hand_mode_)
    {
        case MissionControlHandData::HandModes::kConcurrent:
        HandleConcurrentMovement(speed, thumb_angle, pointer_angle, middle_angle, ring_angle,
                           pinky_angle, roll, pitch);
        break;
      case MissionControlHandData::HandModes::kPitch:
        SetWristPitchPosition(speed, pitch);
        break;
      case MissionControlHandData::HandModes::kRoll:
        SetWristRollPosition(speed, roll);
        break;
      case MissionControlHandData::HandModes::kClose: CloseHand(speed); break;
      case MissionControlHandData::HandModes::kOpen: OpenHand(speed); break;
    }
  }

  int GetWristPitch()
  {
    return wrist_.GetPitchPosition();
  }

  int GetWristRoll()
  {
    return wrist_.GetRollPosition();
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

  MissionControlHandData::HandModes GetCurrentHandMode()
  {
    return current_hand_mode_;
  }

  void SetCurrentHandMode(MissionControlHandData::HandModes new_mode)
  {
    current_hand_mode_ = new_mode;
  }

  void HomeHand(float speed, float rotunda_offset_angle)
  {
    pinky_.Home();
    ring_.Home();
    middle_.Home();
    pointer_.Home();
    thumb_.Home();
    wrist_.Home(speed, float(rotunda_offset_angle));
  }

  void CloseHand(float speed)
  {
    HandleConcurrentMovement(speed, thumb_.GetMaxAngle(), pointer_.GetMaxAngle(),
                       middle_.GetMaxAngle(), ring_.GetMaxAngle(),
                       pinky_.GetMaxAngle(), wrist_.GetRollPosition(),
                       wrist_.GetPitchPosition());
  }

  void OpenHand(float speed)
  {
    HandleConcurrentMovement(speed, thumb_.GetMinAngle(), pointer_.GetMinAngle(),
                       middle_.GetMinAngle(), ring_.GetMinAngle(),
                       pinky_.GetMinAngle(), wrist_.GetRollPosition(),
                       wrist_.GetPitchPosition());
  }

 private:
  MissionControlHandData::HandModes current_hand_mode_ =
      MissionControlHandData::HandModes::kConcurrent;

  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Finger & pinky_;
  sjsu::arm::Finger & ring_;
  sjsu::arm::Finger & middle_;
  sjsu::arm::Finger & pointer_;
  sjsu::arm::Finger & thumb_;
};

}  // namespace sjsu::arm