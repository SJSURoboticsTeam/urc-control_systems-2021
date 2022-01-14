#pragma once

#include "utility/log.hpp"

namespace sjsu::common
{
class Heartbeat
{
  /// Checks if the mission control heartbeat matches rover heartbeat
  bool IsHeartbeatSynced(int mission_control_heartbeat)
  {
    if (heartbeat_count_ != mission_control_heartbeat)
    {
      // TODO: Should throw error in an attempt to reconnect?
      sjsu::LogError("Heartbeat out of sync - resetting!");
      ResetHeartbeat();
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

 private:
  void ResetHeartbeat()
  {
    heartbeat_count_ = 0;
  }

  int heartbeat_count_ = 0;
};
}  // namespace sjsu::common