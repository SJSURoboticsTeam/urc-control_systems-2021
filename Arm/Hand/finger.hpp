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

void initialize()
{
    servo_.ModuleInitialize();
}

void SetPosition(){};

void SetSpeed(){};

void SetMaxAngle(){};

void SetMinAngle(){};

int GetPosition(){};

int GetSpeed(){};

int GetMaxAngle(){};

int GetMinAngle(){};

private:

 sjsu::Servo & servo_;

 float position_  = 0;
 float speed_     = 0;
 float max_angle_ = 0;
 float min_angle_ = 0;

};

}//sjsu::arm