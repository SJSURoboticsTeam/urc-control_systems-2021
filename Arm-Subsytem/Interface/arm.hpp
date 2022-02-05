#pragma once

namespace sjsu::arm
{

class Arm
{
  virtual void Initialize()     = 0;
  virtual void HomeArm()        = 0;
  virtual void HandleMovement() = 0;
};
}  // namespace sjsu::arm