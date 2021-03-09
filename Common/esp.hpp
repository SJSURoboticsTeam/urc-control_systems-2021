#pragma once

#include <string>
#include <cstdint>
#include <string_view>

#include "utility/log.hpp"
#include "peripherals/stm32f10x/uart.hpp"
#include "devices/communication/esp8266.hpp"

namespace sjsu::common
{
/// Esp class manages the esp01/esp8266 WiFi module on the rover
class Esp
{
 public:
  Esp()
      : esp_(sjsu::stm32f10x::GetUart<1>()),
        wifi_(esp_.GetWiFi()),
        socket_(esp_.GetInternetSocket()){};

  /// Initializes the Wi-Fi module by connecting to WiFi
  void Initialize()
  {
    esp_.Initialize();
    ConnectToWiFi();
  };

  /// Verifies that the Wi-Fi module is still connected to the network
  /// @return true if the module is still connected to the internet
  bool isConnectedToWiFi()
  {
    if (!wifi_.IsConnected())
    {
      sjsu::LogError("Lost connection to %s... Reconnecting...", kSsid);
      ConnectToWiFi();
      if (!wifi_.IsConnected())
      {
        sjsu::LogError("Unable to reconnect to %s...", kSsid);
        return false;
      }
    }
    return true;
  };

  /// Sends a GET request to the specified url
  /// @param queryStreamParameters
  /// @return the response body data
  std::string GET(std::string queryStreamParameters)
  {
    std::string host         = kHost + queryStreamParameters;
    std::string_view request = "GET / HTTP/1.1\r\nHost: " + host + "\r\n\r\n";

    socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, host, kPort, 5s);
    std::span request_payload(reinterpret_cast<const uint8_t *>(request.data()),
                              request.size());
    socket_.Write(request_payload, 5s);
    std::array<uint8_t, 1024 * 2> response;
    size_t read_back = socket_.Read(response, 10s);
    // TODO: Return actual GET response
    return "HTTP/1";
  };

 private:
  /// Attempts to connect to the hardcoded WiFi address
  void ConnectToWiFi()
  {
    while (true)
    {
      sjsu::LogInfo("Attempting to connect to %s...", kSsid);
      if (wifi_.ConnectToAccessPoint(kSsid, kPassword, 10s))
      {
        break;
      }
      sjsu::LogError("Failed to connect to %s... Retrying...", kSsid);
      wifi_.DisconnectFromAccessPoint();
    }
    sjsu::LogInfo("Connected to %s", kSsid);
  }

  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;
  const uint16_t kPort        = 5500;
  const std::string kHost     = "http://127.0.0.1:5500/";
  const std::string kSsid     = "network-name";
  const std::string kPassword = "password";
};
}  // namespace sjsu::common
