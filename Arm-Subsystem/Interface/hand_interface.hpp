#pragma once

namespace sjsu::arm
{

class HandInterface
{
    struct MissionControl
  {
    enum class HandModes : char
    {
    
    };

  };
    virtual void Initialize()     = 0;
    virtual void PrintHandData()  = 0;
    virtual void HomeHand(float, float)       = 0;
    virtual void HandleMovement(MissionControl, float) = 0;
    
};
}