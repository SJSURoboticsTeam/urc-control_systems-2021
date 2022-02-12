#pragma once
namespace sjsu::drive
{
  
enum class Modes : char
  {
    DriveMode      = 'D',
    SpinMode       = 'S',
    TranslateMode  = 'T',
    LeftWheelMode  = 'L',
    RightWheelMode = 'R',
    BackWheelMode  = 'B'
  };

class drive{
    virtual void SwitchLegOrientation() = 0;
    virtual bool IsNewMode() = 0;
    virtual Modes GetCurrentMode() = 0;
    virtual void StopWheels() = 0;
    virtual bool IsStopped() = 0;
    virtual void SetWheelSpeed() = 0;
    virtual void HomeWheels() = 0;
    virtual bool AllWheelsAreHomed() = 0;
    virtual void SetMode() = 0;
    virtual void SetDriveMode() = 0;
    virtual void SetSpinMode() = 0;
    virtual void SetTranslationMode() = 0;
    virtual void SetSingleWheelMode() = 0;
    virtual void HandleDriveMode() = 0;
    virtual float GetOutterWheelDriveAngle() = 0;
    virtual float GetBackWheelDriveAngle() = 0;
    virtual void HandleSpinMode() = 0;
    virtual void HandleTranslationMode() = 0;
    virtual void HandleSingularWheelMode() = 0;
}

}  // namespace sjsu::drive