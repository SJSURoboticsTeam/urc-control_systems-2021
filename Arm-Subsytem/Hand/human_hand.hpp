#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
#include "Interface/hand.hpp"
#include "pca9685.hpp"

namespace sjsu::arm
{
class Hand
{
 public:
  struct MissionControlData
  {
    enum class HandModes : char
    {
      kPitch      = 'P',
      kRoll       = 'R',
      kConcurrent = 'C'
    };
    HandModes hand_mode = HandModes::kConcurrent;

    struct FingerAngles
    {
      int pinky_angle   = 0;
      int ring_angle    = 0;
      int middle_angle  = 0;
      int pointer_angle = 0;
      int thumb_angle   = 0;
    };
    FingerAngles finger_angles;

    struct Wrist
    {
      int roll  = 0;
      int pitch = 0;
    };
    Wrist wrist_data;
  };

  Hand(Pca9685 & pca,
       WristJoint & wrist,
       Finger & pinky,
       Finger & ring,
       Finger & middle,
       Finger & pointer,
       Finger & thumb)
      : pca_(pca),
        wrist_(wrist),
        pinky_(pinky),
        ring_(ring),
        middle_(middle),
        pointer_(pointer),
        thumb_(thumb)
  {
  }

  void Initialize()
  {
    pca_.ModuleInitialize();
    wrist_.Initialize();
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

  void HandleConcurrentMovement(MissionControlData::FingerAngles finger_data,
                                MissionControlData::Wrist wrist_data,
                                float speed)
  {
    MoveFinger(finger_data.pinky_angle, pinky_);
    MoveFinger(finger_data.ring_angle, ring_);
    MoveFinger(finger_data.middle_angle, middle_);
    MoveFinger(finger_data.pointer_angle, pointer_);
    MoveFinger(finger_data.thumb_angle, thumb_);
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

  void HandleMovement(MissionControlData hand_data, float speed)
  {
    switch (current_hand_mode_)
    {
      case MissionControlData::HandModes::kConcurrent:
        HandleConcurrentMovement(hand_data.finger_angles, hand_data.wrist_data,
                                 speed);
        break;
      case MissionControlData::HandModes::kPitch:
        SetWristPitchPosition(hand_data.wrist_data.pitch, speed);
        break;
      case MissionControlData::HandModes::kRoll:
        SetWristRollPosition(hand_data.wrist_data.roll, speed);
        break;
    }
  }

  void HomeHand(float rotunda_offset_angle, float speed)
  {
    MoveFinger(pinky_.GetMaxAngle(), pinky_);
    MoveFinger(ring_.GetMaxAngle(), ring_);
    MoveFinger(middle_.GetMaxAngle(), middle_);
    MoveFinger(pointer_.GetMaxAngle(), pointer_);
    MoveFinger(thumb_.GetMaxAngle(), thumb_);
    wrist_.Home(float(rotunda_offset_angle), speed);
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

 private:

  //private member functions
  void MoveFinger(int angle, Finger & finger)
  {
    finger.SetPositionAndPwm(angle);
    pca_.setPulseWidth(finger.GetPwmPin(), finger.GetPWM());
  }

  //private member variables
  MissionControlData::HandModes current_hand_mode_ =
      MissionControlData::HandModes::kConcurrent;
  
  Pca9685 & pca_;

  WristJoint & wrist_;
  Finger & pinky_;
  Finger & ring_;
  Finger & middle_;
  Finger & pointer_;
  Finger & thumb_;
};

}  // namespace sjsu::arm