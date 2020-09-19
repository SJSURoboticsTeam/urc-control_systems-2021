#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task_scheduler.hpp"
#include "L2_HAL/boards/sjtwo.hpp"
#include "utility/log.hpp"
#include "utility/rtos.hpp"

class LEDTask final : public sjsu::rtos::Task<1024>
{
public:
  LEDTask(const char *task_name) : Task(task_name, sjsu::rtos::Priority::kMedium) {}

  bool Setup() override
  {
    sjtwo::led0.SetAsOutput();
    sjtwo::led1.SetAsOutput();

    sjtwo::led0.SetHigh();
    sjtwo::led1.SetLow();
    return true;
  }

  bool Run() override
  {
    sjtwo::led0.Toggle();
    sjtwo::led1.Toggle();
    return true;
  }
};

class PrinterTask final : public sjsu::rtos::Task<1024>
{
public:
  PrinterTask(const char *task_name, const char *message)
      : Task(task_name, sjsu::rtos::Priority::kMedium),
        message_(message),
        run_count_(0)
  {
    print_mutex = NULL;
  }

  bool Setup() override
  {
    if (print_mutex == NULL)
    {
      print_mutex = xSemaphoreCreateMutex();
    }
    sjsu::LogInfo("Completed Setup() for: %s", GetName());
    return true;
  }

  bool PreRun() override
  {
    xSemaphoreTake(print_mutex, portMAX_DELAY);
    sjsu::LogInfo("Completed PreRun() for: %s", GetName());
    xSemaphoreGive(print_mutex);
    return true;
  }

  bool Run() override
  {
    // The text displayed
    run_count_ += 1;
    xSemaphoreTake(print_mutex, portMAX_DELAY);
    sjsu::LogInfo("%s: %lu", message_, run_count_);
    xSemaphoreGive(print_mutex);
    if (run_count_ == 10)
    {
      sjsu::LogInfo("Deleting: %s", GetName());
      Delete();
    }
    return true;
  }

private:
  inline static SemaphoreHandle_t print_mutex;

  const char *message_;
  uint32_t run_count_;
};

sjsu::rtos::TaskScheduler scheduler;
PrinterTask printer_one("Printer A", "Printer A: ");
PrinterTask printer_two("Printer B", "Printer B: ");
LEDTask led_one("Led 1");
LEDTask led_two("Led 2");

int main()
{
  sjsu::LogInfo("Starting TaskScheduler example...");

  scheduler.AddTask(&printer_one);
  scheduler.AddTask(&printer_two);
  scheduler.AddTask(&led_one);
  scheduler.AddTask(&led_two);

  printer_one.SetDelayTime(500);
  printer_two.SetDelayTime(1000);
  led_one.SetDelayTime(5000);
  led_two.SetDelayTime(1000);

  sjsu::LogInfo("Attempting to search for Printer A in the scheduler...");
  sjsu::rtos::TaskInterface *task = scheduler.GetTask("Printer A");

  sjsu::LogInfo("Attempting to search for Led 1 in the scheduler...");
  sjsu::rtos::TaskInterface *task2 = scheduler.GetTask("Led 1");

  sjsu::LogInfo("Attempting to search for Led 1 in the scheduler...");
  sjsu::rtos::TaskInterface *task3 = scheduler.GetTask("Led 2");

  if (task != nullptr)
  {
    sjsu::LogInfo("Found task: %s", task->GetName());
  }
  else
  {
    sjsu::LogError("Could not find task \"Printer A\", Halt System.");
    sjsu::Halt();
  }

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
