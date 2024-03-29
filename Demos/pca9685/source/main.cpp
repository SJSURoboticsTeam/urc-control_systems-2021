#include "../../Arm/pca9685.hpp"
#include "peripherals/lpc40xx/i2c.hpp"

int main(void)
{
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
  sjsu::Pca9685 pca(i2c, 0x70);  // address is from figure 4 of datasheet
  pca.ModuleInitialize();
  sjsu::LogInfo("Starting program");

  while (true)
  {
    pca.setPulseWidth(0, 1_ms);
    sjsu::Delay(1.5_s);
    pca.setPulseWidth(0, 2_ms);

    sjsu::Delay(1.5_s);
    sjsu::LogInfo("ran program");
  }
}
