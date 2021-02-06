#pragma once
#include <string_view>

#include "peripherals/stm32f10x/uart.hpp"
#include "devices/communication/esp8266.hpp"
#include "utility/log.hpp"

namespace sjsu
{
/// WiFi class manages the WiFi modules on the rover.
class WiFi
{
 public:
  WiFi() : {};

  void Initialize();
  void Connect(std::string ssid, std::string_view password);
  void GET(std::string_view url, std::string_view header);
  bool isConnected();

 private:
  std::string ssid;
  std::string password;
};
}  // namespace sjsu
