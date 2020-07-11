#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task_scheduler.hpp"
#include "L2_HAL/actuators/servo/servo.hpp"
#include "utility/log.hpp"
#include "utility/rtos.hpp"

class ServoTask final : public sjsu::rtos::Task<1024>
{
 public:
  ServoTask(const char * task_name, sjsu::lpc40xx::Pwm pwmPin)
      : Task(task_name, sjsu::rtos::Priority::kMedium),
        servo(pwmPin),
        changeServoDirection(false)
  {
  }

  bool Setup() override
  {
    servo.Initialize();
    servo.SetPulseBounds(500us, 2500us);
    servo.SetAngleBounds(0_deg, 180_deg);
    sjsu::LogInfo("Servo setup complete.");
    return true;
  }

  bool Run() override
  {
    if (changeServoDirection)
    {
      servo.SetAngle(10_deg);
      sjsu::LogInfo("Servo set to new angle: 10_deg");
    }
    else
    {
      servo.SetAngle(170_deg);
      sjsu::LogInfo("Servo set to new angle: 170_deg");
    }
    changeServoDirection = !changeServoDirection;
    return true;
  }

 private:
  sjsu::Servo servo;
  bool changeServoDirection;
};

class PrinterTask final : public sjsu::rtos::Task<1024>
{
 public:
  PrinterTask(const char * task_name, const char * message)
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

  const char * message_;
  uint32_t run_count_;
};

sjsu::rtos::TaskScheduler scheduler;
PrinterTask printer_one("Printer A", "Printer A: ");
PrinterTask printer_two("Printer B", "Printer B: ");
//Setting up pwn pin w/ servo
sjsu::lpc40xx::Pwm p2_0(sjsu::lpc40xx::Pwm::Channel::kPwm0);
ServoTask servo_one("Servo 1", p2_0);

int main()
{
  sjsu::LogInfo("Starting TaskScheduler example...");

  scheduler.AddTask(&printer_one);
  scheduler.AddTask(&printer_two);
  scheduler.AddTask(&servo_one);

  // setting Printer A to print 2 times faster than Printer B
  printer_one.SetDelayTime(500);
  printer_two.SetDelayTime(1000);
  //Setting servo delay, hopefully its enough to move between rotations
  servo_one.SetDelayTime(3000);

  sjsu::LogInfo("Attempting to search for Printer A in the scheduler...");
  sjsu::rtos::TaskInterface * task = scheduler.GetTask("Printer A");

  sjsu::LogInfo("Attempting to search for servo_one in the scheduler...");
  sjsu::rtos::TaskInterface * task2 = scheduler.GetTask("Servo 1");

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
    sjsu::LogError("Could not find task Servo 1, Halt System.");
    sjsu::Halt();
  }

  sjsu::LogInfo("Starting scheduler");
  scheduler.Start();
  sjsu::LogInfo("This point should not be reached!");
  return 0;
}
