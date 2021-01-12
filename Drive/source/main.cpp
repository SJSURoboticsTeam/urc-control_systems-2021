#include "utility/units.hpp"
#include "utility/log.hpp"

#include "rover_drive_system.hpp"
#include "wheel.hpp"

int main()
{
  units::angular_velocity::revolutions_per_minute_t kSpeedLimit = 20_rpm;
  sjsu::LogInfo("print rpm", kSpeedLimit.to<double>());

  return 0;
}
