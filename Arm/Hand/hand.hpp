#pragma once
#include "peripherals/uart.hpp"
#include "wrist_joint.hpp"
#include "finger.hpp"
#include "devices/actuators/servo/servo.hpp"

namespace sjsu::arm
{
class Hand
{
  /// The hand has its own MCU that communicates with the arm via UART.
 public:
  //Hand(Uart & uart) : uart_(uart) {}

      struct Fingers
    {
      float pinky_angle_   = 0;
      float ring_angle_    = 0;
      float middle_angle_  = 0;
      float pointer_angle_ = 0;
      float thumb_angle_   = 0;
    };
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
  };

   // can't home yet
  void HomeRoll(){};
  
  //Get finger position
    void SetThumbPosition(float target_thumb_angle)
  { 
    fingers_.thumb_angle_ = float(std::clamp(target_thumb_angle, min_angle, max_angle));
    units::angle::degree_t angle_to_degrees(target_thumb_angle);
    thumb_.SetAngle(angle_to_degrees);
  };

    void SetMiddlePosition(float target_middle_angle)
  {
    fingers_.middle_angle_ = float(std::clamp(target_middle_angle, min_angle, max_angle));
    units::angle::degree_t angle_to_degrees(target_middle_angle);
    middle_.SetAngle(angle_to_degrees);
  };

  void SetPinkyPosition(float target_pinky_angle)
  {
    fingers_.pinky_angle_ = float(std::clamp(target_pinky_angle, min_angle, max_angle));
    units::angle::degree_t angle_to_degrees(target_pinky_angle);
    pinky_.SetAngle(angle_to_degrees);
  };

  void SetPointerPosition(float target_pointer_angle)
  {
    fingers_.pointer_angle_ = float(std::clamp(target_pointer_angle, min_angle, max_angle));
    units::angle::degree_t angle_to_degrees(target_pointer_angle);
    pointer_.SetAngle(angle_to_degrees);
  } ;

  void SetRingPosition(float target_ring_angle)
  {
    fingers_.ring_angle_ = float(std::clamp(target_ring_angle, min_angle, max_angle));
    units::angle::degree_t angle_to_degrees(target_ring_angle);
    ring_.SetAngle(angle_to_degrees);
  };
 
  //Set Finger Position
  int GetThumbPosition()
  {
    return int(fingers_.thumb_angle_);
  };

  int GetMiddlePosition()
  {
    return int(fingers_.middle_angle_);
  };

  int GetPinkyPosition()
  {
    return int(fingers_.pinky_angle_);
  };

  int GetPointerPosition()
  {
    return int(fingers_.pointer_angle_);
  };

  int GetRingPosition()
  {
    return int(fingers_.ring_angle_);
  };
  sjsu::arm::WristJoint & wrist_;
  sjsu::arm::Finger & pinky_;
  sjsu::arm::Finger & ring_;
  sjsu::arm::Finger & middle_;
  sjsu::arm::Finger & pointer_;
  sjsu::arm::Finger & thumb_;

  const float min_angle  = 0;
  const float max_angle  = 180;
  const float rest_angle = 90;

  Fingers fingers_;
  private:
  // Uart & uart_;
};

}  // namespace sjsu::arm