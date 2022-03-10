#pragma once
#include <string>

#include "../Common/heartbeat.hpp"

namespace sjsu::common
{
class RoverSystemInterface : public Heartbeat
{
 public:
  struct RoverMissionControlData
  {
    int is_operational  = 0;
    int heartbeat_count = 0;
  };
  virtual void Initialize()                                        = 0;
  virtual void PrintRoverData()                                    = 0;
  virtual std::string CreateGETRequestParameterWithRoverStatus()   = 0;
  virtual void ParseMissionControlCommands(std::string & response) = 0;
  virtual void HandleRoverCommands()                               = 0;

  bool IsOperational(int is_operational)
  {
    if (is_operational != 1)
    {
      sjsu::LogWarning("Rover is not operational!");
      return false;
    }
    return true;
  }

 protected:
  Heartbeat heartbeat_;
};
}  // namespace sjsu::common