#pragma once

namespace sjsu::arm
{

class Arm
{

    virtual void Initalize() = 0;

    virtual void HomeArm() = 0;

    virtual void HandleMovement() = 0;
}; 
}//sjsu::arm