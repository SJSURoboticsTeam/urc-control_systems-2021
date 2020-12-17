#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

namespace sjsu
{
class RoverDriveSystems
{
  public:
  // Three different driving modes
  void handleDriveMode(float speed, units::angle::degree_t angle);          // Normal driving mode
  void handleTranslationMode(float speed, units::angle::degree_t angle);    // Side to side movement
  void handleSpinMode(float speed, units::angle::degree_t angle);           // Spin in place (turning)
  
  // Initialization handlers
  RoverDriveSystems();
  void Initialize();
  void handleNewMode(char mode);

  private:
  // Creating PWM on pin 2.0, 2.1, 2.2 
  // Might not need - just handle within setup/initialization process
  sjsu::lpc40xx::Pwm pwm1;
  sjsu::lpc40xx::Pwm pwm2;
  sjsu::lpc40xx::Pwm pwm3;

  // Create servo steering classes
  sjsu::Servo steeringLeftWheel;
  sjsu::Servo steeringRightWheel;
  sjsu::Servo steeringBackWheel;

  // Create servo drive classes
  sjsu::Servo driveLeftWheel;
  sjsu::Servo driveRightWheel;
  sjsu::Servo driveBackWheel;

};
}  // namespace sjsu