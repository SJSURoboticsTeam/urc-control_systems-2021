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
    void SwitchLegOrientation();
    bool IsNewMode();
    Modes GetCurrentMode();
    void StopWheels();
    bool IsStopped();
    void SetWheelSpeed();
    void HomeWheels();
    bool AllWheelsAreHomed();
    void SetMode();
    void SetDriveMode();
    void SetSpinMode();
    void SetTranslationMode();
    void SetSingleWheelMode();
    void HandleDriveMode();
    float GetOutterWheelDriveAngle();
    float GetBackWheelDriveAngle();
    void HandleSpinMode();
    void HandleTranslationMode();
    void HandleSingularWheelMode();
};

}  // namespace sjsu::drive