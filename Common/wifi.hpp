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

  /// Initializes the WiFi module
  void Initialize();

  /// Connects the WiFi module to the network
  /// @param ssid name of the network
  /// @param password netork password
  void Connect(std::string ssid, std::string_view password);

  /// Verifies that the WiFi module is still connected to the network
  /// @return true if the module is still connected to the internet
  bool isConnected();

  /// Sends a GET request to the specified url
  /// @param url the requested url address or endpoint
  /// @return the response body data
  std::string GET(std::string_view url);

 private:
  std::string ssid_;
  std::string password_;
};
}  // namespace sjsu
