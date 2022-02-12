#pragma once
namespace sjsu::drive
{

class drive{
    virtual void SwitchLegOrientation() = 0;
    virtual bool IsNewMode()
    virtual Modes GetCurrentMode()
    virtual void StopWheels()
    virtual bool IsStopped()
    virtual void SetWheelSpeed()
    virtual void HomeWheels()
    virtual bool AllWheelsAreHomed()
    virtual void SetMode()
    virtual void SetDriveMode()
    
}

}  // namespace sjsu::drive