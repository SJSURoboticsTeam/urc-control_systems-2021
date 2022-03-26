
#include "devices/sensors/battery/max17043.hpp"
#include "peripherals/lpc17xx/i2c.hpp"
#include "peripherals/lpc40xx/gpio.hpp"
#include "peripherals/hardware_counter.hpp"
#include "devices/sensors/battery/ltc4150.hpp" 
namespace sjsu::common
{
/// State of charge manages ltc4150 and max17043
class StateOfCharge
{
public:
  StateOfCharge(){}
void Initialize(){
  //TODO - impliment initialize for state of charge
}

double StateOfCharge_MAX(){
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
  i2c.Initialize();
  sjsu::lpc40xx::Gpio & alert_pin = sjsu::lpc40xx::GetGpio<0, 6>();
  alert_pin.Initialize();
  bool callback_was_called   = false;
        InterruptCallback callback = [&callback_was_called]() {
          callback_was_called = true;
        };
  uint8_t address = 0b0110110;

  Max170343 *battery_sensor = new Max170343(i2c, alert_pin, callback, address);

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

    voltage = (voltage-3)/(4.2 - 3);

  sjsu::LogInfo("Max: %f", voltage);
  return voltage;

}

double StateOfCharge_LTC(){
  
  sjsu::lpc40xx::Gpio &gpio = sjsu::lpc40xx::GetGpio<0, 1>();
  sjsu::GpioCounter counter(gpio, sjsu::Gpio::Edge::kRising);

  sjsu::lpc40xx::Gpio &pol = sjsu::lpc40xx::GetGpio<2, 3>();
  units::impedance::ohm_t resistance = 10'000_Ohm;

  Ltc4150 *battery_sensor = new Ltc4150(
                             counter,
                             pol,
                             resistance);

  units::charge::microampere_hour_t battery_charge = battery_sensor->GetCharge();
  double charge = battery_charge.to<double>();
  return charge/100.0; // 100 is a temporary and will need to be updated when we know the battery mA/h

}

  };
}  // namespace sjsu::common

