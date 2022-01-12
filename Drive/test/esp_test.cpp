#include "testing/testing_frameworks.hpp"
#include "devices/communication/esp8266.hpp"
#include "peripherals/lpc40xx/uart.hpp"

#include "../Common/esp.hpp"

namespace sjsu
{
TEST_CASE("Esp testing")
{
  Mock<Uart> mock_uart;
  Mock<WiFi> mock_wifi;
  Mock<InternetSocket> mock_socket;
  common::Esp esp(mock_uart.get(), mock_wifi.get(), mock_socket.get());
  Mock<Esp> spy_esp(esp);

  SECTION("Test") {}
}
}  // namespace sjsu