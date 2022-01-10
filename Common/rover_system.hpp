#pragma once
#include <string>

namespace sjsu::common
{
class RoverSystem
{
 public:

  /// Initialize all the motors and sensors that are used in the system
  virtual void Initialize() = 0;
  /// Prints the status of all the rover devices & motors
  virtual void PrintRoverData() = 0;
  /// Creates the GET request parameters that contain the current rover status
  virtual std::string GETParameters() = 0;
  /// Parses the JSON response retrieved from mission control
  virtual void ParseJSONResponse(std::string &response) = 0;
  /// Move the rover according to the data sent from mission control
  virtual void HandleRoverMovement() = 0;
};
}  // namespace sjsu::common