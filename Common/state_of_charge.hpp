#include "devices/sensors/battery/max17043.hpp"
#include "peripherals/lpc17xx/i2c.hpp"
#include "peripherals/lpc40xx/gpio.hpp"
#include "peripherals/hardware_counter.hpp"
#include "devices/sensors/battery/ltc4150.hpp"
#include "Interface/state_of_charge_interface.hpp"

namespace sjsu::common
{
/// State of charge manages ltc4150 and max17043
class StateOfCharge
{
 public:
  StateOfCharge() {}
  void Initialize()
  {
    // TODO - impliment initialize for state of charge
    sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
    i2c.Initialize();
    sjsu::lpc40xx::Gpio & alert_pin = sjsu::lpc40xx::GetGpio<0, 6>();
    alert_pin.Initialize();
    bool callback_was_called   = false;
    InterruptCallback callback = [&callback_was_called]()
    { callback_was_called = true; };
    uint8_t address = 0b0110110;

    Max170343 * battery_sensor =
        new Max170343(i2c, alert_pin, callback, address);

    units::voltage::volt_t battery_voltage = battery_sensor->GetVoltage();
    double voltage                         = battery_voltage.to<double>();
  }

  double StateOfCharge_MAX()
  {
    if (voltage < 3.05)
    {
      sjsu::LogWarning("Cell Voltage below 3. Battery dangerously low!");
    }
    else if (voltage < 3.25)
          InterruptCallback callback = [&callback_was_called]() {
            callback_was_called = true;
          };
    uint8_t address = 0b0110110;

    Max170343 *battery_sensor = new Max170343(i2c, alert_pin, callback,
    address);

    units::voltage::volt_t battery_voltage = battery_sensor -> GetVoltage();
    double voltage = battery_voltage.to<double>();

    if(voltage < 3.05)
    {
      sjsu::LogWarning("Cell Voltage below 3. Battery dangerously low!");
    }
    else if(voltage < 3.25 )
    {
      sjsu::LogWarning("Cell Voltage below 3.25. Battery is low");
    }

    voltage = (voltage - 3) / (4.2 - 3);

    voltage = voltage * 100;
  }

  float GetStateOfCharge()
  {
    return voltage;
  } 

};
}  // namespace sjsu::common