#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task_scheduler.hpp"
#include "L2_HAL/boards/sjtwo.hpp"
#include "utility/log.hpp"
#include "utility/rtos.hpp"

// Super basic RTOS demo application.

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
    sjtwo::led1.SetLow();
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

// 1. Creating another Task object that inherits from rtos Task.
class OLEDTask final : public sjsu::rtos::Task<1024>
{
public:
  // Constructor for the rtos task. It needs a task_name argument and a scheduler priority level ranging from low to high.
  OLEDTask(const char *task_name) : Task(task_name, sjsu::rtos::Priority::kMedium)
  {
    sjsu::Graphics &oled_graphics = sjtwo::Oled();
    sjsu::TerminalCache_t<
        sjsu::Ssd1306::kHeight / sjsu::GraphicalTerminal::kCharacterHeight,
        sjsu::Ssd1306::kWidth / sjsu::GraphicalTerminal::kCharacterWidth>
        cache;
    sjsu::GraphicalTerminal oled_terminal(&oled_graphics, &cache);
  }
  // Setup() and Run() are required functions within the rtos class. Setup() will be run initially during instanstiation.
  bool Setup() override
  {
    oled_terminal.Initialize();
    return true;
  }
  // Run() will be the instrutctions to be ran over and over again.
  bool Run() override
  {
    oled_terminal.printf("Float: %.1f\nInteger: %d", 234.5, 15);
    sjsu::Delay(3000ms);
    oled_terminal.Clear();
    return true;
  }
};

// 2. Declare a TaskScheduler object. It is responsible for keeping track of the tasks.
sjsu::rtos::TaskScheduler scheduler;
// 3. Declare the Task you wrote earlier. In this case I have two LED and 1 OLED objects for demonstration purposes.
LEDTask led_one("Led 1");
LEDTask led_two("Led 2");
OLEDTask oled_one("Oled 1");

int main()
{
  sjsu::LogInfo("Starting TaskScheduler example...");
  // 3. Add the Tasks to the scheduler. Think of it like loading in tasks to be managed by the scheduler.
  scheduler.AddTask(&led_one);
  scheduler.AddTask(&led_two);
  // 4. Can adjust the time alloted to each task between switching off from each other.
  led_one.SetDelayTime(5000);
  led_two.SetDelayTime(1000);
  // 5.
  sjsu::LogInfo("Attempting to search for Led 1 in the scheduler...");
  sjsu::rtos::TaskInterface *task2 = scheduler.GetTask("Led 1");

  sjsu::LogInfo("Attempting to search for Led 1 in the scheduler...");
  sjsu::rtos::TaskInterface *task3 = scheduler.GetTask("Led 2");

  if (task2 != nullptr)
  {
    sjsu::LogInfo("Found task: %s", task2->GetName());
  }
  else
  {
    sjsu::LogError("Could not find task led 1, Halt System.");
    sjsu::Halt();
  }

  if (task3 != nullptr)
  {
    sjsu::LogInfo("Found task: %s", task3->GetName());
  }
  else
  {
    sjsu::LogError("Could not find task led 2, Halt System.");
    sjsu::Halt();
  }

  sjsu::LogInfo("Starting scheduler");
  scheduler.Start();
  sjsu::LogInfo("This point should not be reached!");
  return 0;
}
