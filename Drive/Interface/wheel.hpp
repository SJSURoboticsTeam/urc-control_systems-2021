#pragma once

namespace sjsu::drive
{
class Wheel
{
 public:
  virtual void Initialize()         = 0;
  virtual void Print()              = 0;
  virtual std::string GetName()     = 0;
  virtual int GetHubSpeed()         = 0;
  virtual int GetSteerAngle()       = 0;
  virtual void SetHubSpeed(float)   = 0;
  virtual void SetSteerAngle(float) = 0;
  virtual bool IsHomed()            = 0;
};
}  // namespace sjsu::drive