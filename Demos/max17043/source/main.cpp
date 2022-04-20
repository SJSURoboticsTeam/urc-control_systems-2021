#include "utility/log.hpp"
#include "../../Common/state_of_charge.hpp"

int main(void)
{
  while (1)
  {
    try
    {
      sjsu::common::StateOfCharge st;
      sjsu::Delay(1000ms);
      sjsu::LogInfo("Max: %f%% remaining", st.GetStateOfCharge());
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Uncaught error in main() - Stopping Rover!");
    }
  }

  return 0;
}