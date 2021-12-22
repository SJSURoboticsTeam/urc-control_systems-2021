#include "library/devices/sensors/battery/max17043.hpp"
#include "library/peripherals/lpc17xx/i2c.hpp"
#include "library/peripherals/lpc40xx/gpio.hpp"
#include "library/peripherals/hardware_counter.hpp"
#include "library/devices/sensors/battery/ltc4150.hpp" 
namespace sjsu::common
{
/// State of charge manages ltc4150 and max17043
class StateOfCharge
{
public:
  StateOfCharge()
      : StateOfCharge_(sjsu::lpc40xx::GetUart<4>()),

double StateOfCharge_MAX(){
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<0>();
  i2c.Initialize();
  sjsu::lpc40xx::Gpio & alert_pin = sjsu::lpc40xx::GetGpio<0, 1>();
  alert_pin.Initialize();
  bool callback_was_called   = false;
        InterruptCallback callback = [&callback_was_called]() {
          callback_was_called = true;
        };
  uint8_t address = 0b0110110;

  Max170343 *batterySensor = new Max170343(i2c, alert_pin, callback, address);

  units::voltage::volt_t batteryVoltage = batterySensor -> GetVoltage();
  double Voltage = batteryVoltage.to<double>();
  return Voltage/3.3;

}

double StateOfCharge_LTC(){
  
  sjsu::lpc40xx::Gpio &gpio = sjsu::lpc40xx::GetGpio<0, 1>();
  sjsu::GpioCounter counter(gpio, sjsu::Gpio::Edge::kRising);

  sjsu::lpc40xx::Gpio &pol = sjsu::lpc40xx::GetGpio<2, 3>();
  units::impedance::ohm_t resistance = 10'000_Ohm;

  Ltc4150 *batterySensor = new Ltc4150(
                             counter,
                             pol,
                             resistance);

  units::charge::microampere_hour_t batteryCharge = batterySensor->GetCharge();
  double Charge = batteryCharge.to<double>();
  return Charge/100.0; // 100 is a temporary and will need to be updated when we know the battery mA/h

}

  };
}  // namespace sjsu::common