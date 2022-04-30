#pragma once

namespace sjsu::common
{
 class StateOfChargeInterface
  {
  public:
    virtual void CalcState() = 0;
    virtual bool IsBatteryLow() = 0;
    virtual float GetStateOfCharge() = 0;
  };

}
