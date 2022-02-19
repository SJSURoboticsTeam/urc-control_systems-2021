#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "Interface/hand.hpp"
#include "pca9685.hpp" 
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
       sjsu::arm::Finger & thumb,
       sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>(),
       sjsu::Pca9685 pca(i2c)),
       int pin_number,
       float min_pulse,
       float max_pulse,
       float position,
       float speed,
       float max_angle,
       float min_angle)
      : wrist_(wrist),
        pinky_(pinky),
        ring_(ring),
        middle_(middle),
        pointer_(pointer),
        thumb_(thumb),
        pca_(pca),
        pin_number_(pin_number),
        position_(position),
        speed_(speed),
        max_angle_(max_angle),
        min_angle_(min_angle),
        min_pulse_(min_pulse),
        max_pulse_(max_pulse)
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
    pca_.ModuleInitialize();
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
    thumb_.SetPosition(finger_data.thumb_angle, 0);
    pointer_.SetSpeed(speed);
    pointer_.SetPosition(finger_data.pointer_angle, 1);
    middle_.SetSpeed(speed);
    middle_.SetPosition(finger_data.middle_angle, 2);
    ring_.SetSpeed(speed);
    ring_.SetPosition(finger_data.ring_angle, 3);
    pinky_.SetSpeed(speed);
    pinky_.SetPosition(finger_data.pinky_angle, 4);
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
        HandleConcurrentMovement(hand_data.fingers, hand_data.wrist_data,
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

  void Home()
  {
    return;
  }

  void SetSpeed(float target_speed)
  {
    speed_ = target_speed;
  }

  void SetPosition(float angle, int pin_number_)
  {
    position_ = sjsu::Map(angle, min_angle_, max_angle_, min_pulse_, max_pulse_);
    units::chrono::milliseconds position_(position_);
    // units::angle::degree_t angle_to_degrees(position_);
    // servo_.SetAngle(angle_to_degrees);
    pca_.setPulseWidth(pin_number_,position_);
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
  sjsu::lpc40xx::I2c & i2c_;
  sjsu::Pca9685 pca_;
  float position_  = 0;
  float speed_     = 0;
  float max_angle_ = 180;
  float min_angle_ = 0;
  float min_pulse_ = 2.0;
  float max_pulse_ = 1.0;
  int pin_number_ = 0;
};

}  // namespace sjsu::arm