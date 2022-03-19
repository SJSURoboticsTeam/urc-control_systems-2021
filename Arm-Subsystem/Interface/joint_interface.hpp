

namespace sjsu::arm
{
class joint_interface
{
  struct accelerometer
  {
  } 
  virtual void Initialize()             = 0;
  virtual void SetPosition(float angle) = 0;
  virtual int GetSpeed()                = 0;
  virtual int GetPosition()             = 0;
}
}  // namespace sjsu::arm