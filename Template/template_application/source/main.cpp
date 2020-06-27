#include <cstdint>

#include "L2_HAL/actuators/servo/servo.hpp"
#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task.hpp"
#include "utility/rtos.hpp"
#include "utility/log.hpp"
#include "utility/time.hpp"

// template <class T>
// T GetMissionControlData()
// {
//   return T{};
// }

void exampleServoFunction(void * params)
{
  sjsu::lpc40xx::Pwm pin2_1(sjsu::lpc40xx::Pwm : Channel::kPwm1);
  sjsu::Servo sg90(pin2_1);
  sg90.Initialize();

  while (true)
  {
    sg90.SetAngle(45_deg);
    vTaskDelay(500);
    sg90.SetAngle(-45_deg);
    vTaskDelay(500);
  }
}

int main()
{

  xTaskCreate(exampleServoFunction, "example_servo_task_name",
              sjsu::rtos::StackSize(1024), sjsu::rtos::kNoParameter,
              sjsu::rtos::Priority::kMedium, sjsu::rtos::kNoHandle);
}
