#pragma once
#include <string>

namespace sjsu::common
{
class RoverSystem
{
 public:
  struct RoverMissionControlData{
    int is_operational = 0;
    int heartbeat_count;
  };
  /// Initialize all the motors and sensors that are used in the system
  void Initialize();
  /// Prints the status of all the rover devices & motors
  void PrintRoverData();
  /// Creates the GET request parameters that contain the current rover status
  std::string GETParameters();
  /// Parses the JSON response retrieved from mission control
  std::string ParseJSONResponse();
  /// Move the rover according to the data sent from mission control
  void HandleRoverMovement();
};
}  // namespace sjsu::common