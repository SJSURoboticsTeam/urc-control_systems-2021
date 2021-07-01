#pragma once

#include <string>
#include <cstdint>
#include <string_view>

#include "utility/debug.hpp"
#include "utility/log.hpp"
#include "peripherals/lpc40xx/uart.hpp"
#include "devices/communication/esp8266.hpp"

namespace sjsu::common
{
/// Esp class manages the esp01/esp8266 WiFi module on the rover
class Esp
{
 public:
  Esp()
      : esp_(sjsu::lpc40xx::GetUart<3>()),
        wifi_(esp_.GetWiFi()),
        socket_(esp_.GetInternetSocket()){};

  /// Initializes the Wi-Fi module by connecting to WiFi
  void Initialize()
  {
    sjsu::LogInfo("Initializing Wi-Fi module...");
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
  /// @param endpoint
  /// @return the response body data
  std::string_view GET(std::string endpoint)
  {
    request_ = "GET /" + endpoint + " HTTP/1.1\r\nHost: " + url_.data() +
               "\r\nContent-Type: application/json\r\n\r\n";

    sjsu::LogInfo("Connecting to %s...", url_.data());
    socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, url_, kPort,
                    kDefaultTimeout);

    sjsu::LogInfo("Writing request to server...");
    std::span write_payload(reinterpret_cast<const uint8_t *>(request_.data()),
                            request_.size());
    socket_.Write(write_payload, kDefaultTimeout);

    sjsu::LogInfo("Reading back response from server...");
    std::array<uint8_t, 1024 * 2> response;
    size_t read_back = socket_.Read(response, kDefaultTimeout);

    sjsu::LogInfo("Parsing response body for JSON...");
    std::string_view body(reinterpret_cast<char *>(response.data()), read_back);
    body = body.substr(body.find("\r\n\r\n"));
    body = body.substr(body.find("{"));
    return body.data();
  };

 private:
  /// Attempts to connect to the hardcoded WiFi address
  void ConnectToWiFi()
  {
    while (true)
    {
      sjsu::LogInfo("Attempting to connect to %s...", kSsid);
      if (wifi_.ConnectToAccessPoint(kSsid, kPassword, kDefaultTimeout))
      {
        break;
      }
      sjsu::LogError("Failed to connect to %s... Retrying...", kSsid);
      wifi_.DisconnectFromAccessPoint();
    }
    sjsu::LogInfo("Connected!");
  }

  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;
  std::string_view request_;
  std::string_view url_  = "jsonplaceholder.typicode.com";
  const uint16_t kPort   = 80;
  const char * kSsid     = "GarzaLine";
  const char * kPassword = "NRG523509";
  const std::chrono::nanoseconds kDefaultTimeout = 10s;
};
}  // namespace sjsu::common
