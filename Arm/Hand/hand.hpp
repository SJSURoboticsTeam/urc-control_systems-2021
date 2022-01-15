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
  void SetThumbPosition(float thumb_angle)
  { 
    fingers_.thumb_angle_ = float(std::clamp(thumb_angle, max_angle, min_angle));
    units::angle::degree_t angle_to_degrees(thumb_angle);
    thumb_motor.SetAngle(angle_to_degrees);
  };

  void SetMiddlePosition(double middle_angle)
  {
    fingers_.middle_angle_ = float(std::clamp(thumb_angle, max_angle, min_angle));
    units::angle::degree_t angle_to_degrees(middle_angle);
    middle_motor.SetAngle(angle_to_degrees);
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
    return int();
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
  sjsu::arm::WristJoint wrist_;

 private:
  sjsu::RmdX & thumb_motor_;
  sjsu::RmdX & middle_motor_;
  sjsu::RmdX & pinky_motor_;
  sjsu::RmdX & pointer_motor_;
  sjsu::RmdX & ring_motor_;



 Fingers fingers_;
  // Uart & uart_;
};

}  // namespace sjsu::arm