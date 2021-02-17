#pragma once

#include <string>

#include "peripherals/stm32f10x/uart.hpp"
#include "devices/communication/esp8266.hpp"

namespace sjsu::common
{
/// Esp class manages the esp01/esp8266 Wi-Fi module on the rover.
class Esp
{
 private:
  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;
  const std::string kSsid     = "network-name";
  const std::string kPassword = "password";

 public:
  Esp()
      : esp_(sjsu::stm32f10x::GetUart<1>()),
        wifi_(esp_.GetWiFi()),
        socket_(esp_.GetInternetSocket()){};

  /// Initializes the Wi-Fi module
  void Initialize(){
    // TODO: ConnectToAccessPoint()
  };

  /// Verifies that the Wi-Fi module is still connected to the network
  /// @return true if the module is still connected to the internet
  bool isConnected();

  /// Sends a GET request to the specified url
  /// @param temp_holder
  /// @return the response body data
  std::string GET(std::string temp_holder){
    // TODO: Look into lambdas, there is a way to make this nice
  };
};
}  // namespace sjsu::common
