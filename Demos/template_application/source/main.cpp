#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task_scheduler.hpp"
#include "L2_HAL/boards/sjtwo.hpp"
#include "utility/log.hpp"
#include "utility/rtos.hpp"

// Super basic RTOS demo application using the SJTwo onboard LEDs.(October 2020)

// 1. Create Task object that inherits from rtos Task.
class LEDTask final : public sjsu::rtos::Task<1024>
{
public:
  // Constructor for the rtos task. It needs a task_name argument and a scheduler priority level ranging from low to high.
  LEDTask(const char *task_name) : Task(task_name, sjsu::rtos::Priority::kMedium) {}
  // Setup() and Run() are required functions within the rtos class.
  // Setup() will be run initially during instanstiation.
  bool Setup() override
  {
    sjtwo::led0.SetAsOutput();
    sjtwo::led1.SetAsOutput();

    sjtwo::led0.SetHigh();
    sjtwo::led1.SetHigh();
    return true;
  }
  // Run() will be the instrutctions to be ran over and over again.
  bool Run() override
  {
    sjtwo::led0.Toggle();
    sjtwo::led1.Toggle();
    return true;
  }
};

// Creating another Task object that inherits from rtos Task.
class LEDTaskAlt final : public sjsu::rtos::Task<1024>
{
public:
  // Constructor for the rtos task. It needs a task_name argument and a scheduler priority level ranging from low to high.
  LEDTaskAlt(const char *task_name) : Task(task_name, sjsu::rtos::Priority::kMedium) {}
  // Setup() and Run() are required functions within the rtos class.
  // Setup() will be run initially during instanstiation.
  bool Setup() override
  {
    sjtwo::led2.SetAsOutput();
    sjtwo::led3.SetAsOutput();

    sjtwo::led2.SetHigh();
    sjtwo::led3.SetHigh();
    return true;
  }
  // Run() will be the instrutctions to be ran over and over again.
  bool Run() override
  {
    sjtwo::led2.Toggle();
    sjtwo::led3.Toggle();
    return true;
  }
};

// 2. Declare a TaskScheduler object. It is responsible for keeping track of the tasks.
sjsu::rtos::TaskScheduler scheduler;
// 3. Declare the Tasks listed earlier. In this case I have two LED tasks for demonstration purposes.
LEDTask led_zero("Least Significant LEDs");
LEDTaskAlt led_two("Most Significant LEDs");

int main()
{
  sjsu::LogInfo("Starting TaskScheduler example...");
  // 3. Add the Tasks to the scheduler. Think of it like loading in tasks to be managed by the scheduler.
  scheduler.AddTask(&led_zero);
  scheduler.AddTask(&led_two);
  // // 4. Can adjust the time alloted to each task between switching off from each other.
  // Play around with it. Basically the longer the delay the longer the task will be carried out / LEDs toggled on/off
  led_zero.SetDelayTime(3000);
  led_two.SetDelayTime(5000);

  // 5. Demonstrates how to verify that a task is assigned to the scheduler.
  // Assigns a pointer to the task in the scheduler. Uses the name in order to identify the task in GetTask("name of task")
  sjsu::LogInfo("Attempting to search for Least Significant LEDs in the scheduler...");
  sjsu::rtos::TaskInterface *LeastSigLEDTask = scheduler.GetTask("Least Significant LEDs");
  // If the ptr is null that means the task was not found -> an error has occured
  if (LeastSigLEDTask != nullptr)
  {
    sjsu::LogInfo("Found task: %s", LeastSigLEDTask->GetName());
  }
  else
  {
    sjsu::LogError("Could not find task Least Significant LEDs, Halt System.");
    sjsu::Halt();
  }

  sjsu::LogInfo("Starting scheduler");
  // 6. LAST BUT NOT LEAST - Start the scheduler.
  // Without calling the Start() the tasks will never be called and program would terminate normally/no errors
  scheduler.Start();
  sjsu::LogInfo("This point should not be reached!");
  return 0;
}
