#pragma once

#include "devices/actuators/servo/servo.hpp"

namespace sjsu::arm
{
class Finger
{
public:

Finger(sjsu::Servo & servo) : servo_(servo)
{}

Finger(sjsu::Servo & servo,
        float position,
        float speed,
        float max_angle,
        float min_angle
       ) : servo_(servo), position_(position), speed_(speed), max_angle_(max_angle), min_angle_(min_angle)
       {}

void Initialize()
{
    servo_.ModuleInitialize();
}

void SetPosition(float angle)
{
    position_ = std::clamp(angle, min_angle_, max_angle_);
    units::angle::degree_t angle_to_degrees(position_);
    servo_.SetAngle(angle_to_degrees);
};

void SetSpeed(float target_speed)
{
    speed_ = target_speed;
};

int GetPosition()
{
    return int(position_);
};

int GetSpeed()
{
    return int(speed_);
};


private:

 sjsu::Servo & servo_;

 float position_  = 0;
 float speed_     = 0;
 float max_angle_ = 0;
 float min_angle_ = 0;

};

}//sjsu::arm