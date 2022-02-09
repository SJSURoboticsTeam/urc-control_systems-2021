#pragma once
#include <string>

#include "../Common/heartbeat.hpp"

namespace sjsu::common
{
class RoverSystem
{
 public:
  struct RoverMissionControlData
  {
    int is_operational  = 0;
    int heartbeat_count = 0;
  };
  /// Initialize all the motors and sensors that are used in the system
  virtual void Initialize() = 0;
  /// Prints the status of all the rover devices & motors
  virtual void PrintRoverData() = 0;
  /// Creates the GET request parameters that contain the current rover status
  virtual std::string GETParameters() = 0;
  /// Parses the JSON response retrieved from mission control
  virtual void ParseJSONResponse(std::string & response) = 0;
  /// Move the rover according to the data sent from mission control
  virtual void HandleRoverMovement() = 0;

  bool IsHeartbeatSynced(int heartbeat_count)
  {
    if (heartbeat_count_ != heartbeat_count)
    {
      // TODO: Should throw error in an attempt to reconnect?
      sjsu::LogError("Heartbeat out of sync - resetting!");
      ResetHeartbeatCount();
      return false;
    }
    return true;
  }

  int GetHeartbeatCount()
  {
    return heartbeat_count_;
  }

  void IncrementHeartbeatCount()
  {
    heartbeat_count_++;
  }

  bool IsOperational(int is_operational)
  {
    if (is_operational != 1)
    {
      sjsu::LogWarning("Rover is not operational!");
      return false;
    }
    return true;
  }

 private:
  void ResetHeartbeatCount()
  {
    heartbeat_count_ = 0;
  }

  int heartbeat_count_ = 0;
};
}  // namespace sjsu::common