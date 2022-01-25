#pragma once

#include <string_view>
#include <algorithm>

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
    sjsu::LogInfo("Initializing esp module...");
    esp_.Initialize();
    ConnectToWifi();
    ConnectToServer();
  };

  /// Sends a GET request to the hardcoded URL
  /// @param endpoint i.e. /endpoint?example=parameter
  /// @return the response body of the GET request
  std::string GET(std::string endpoint)
  {
    request_ = "GET /" + endpoint + " HTTP/1.1\r\nHost: " + url_ + "\r\n\r\n";
    WriteToServer();
    try
    {
      std::array<uint8_t, 1024 * 2> raw;
      std::fill(raw.begin(), raw.end(), 0);
      size_t read_back = socket_.Read(raw, kDefaultTimeout);
      std::string response(reinterpret_cast<char *>(raw.data()), read_back);
      try
      {
        response = response.substr(response.find("\r\n\r\n{"));
        printf("Response Body:%s", response.c_str());
        return response;
      }
      catch (const std::exception & e)
      {
        sjsu::LogError("Error parsing response from server!");
        return kErrorResponse;
      }
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error reading response from server!");
      return kErrorResponse;
    }
  };

  /// Attempts to connect to the local WiFi network
  void ConnectToWifi()
  {
    while (true)
    {
      sjsu::LogInfo("Attempting to connect to %s...", kSsid);
      if (wifi_.ConnectToAccessPoint(kSsid, kPassword, kDefaultTimeout))
      {
        break;
      }
      sjsu::LogWarning("Connecting...", kSsid);
    }
  }

  void IsServerExpired(sjsu::TimeoutTimer & serverTimer)
  {
    if (serverTimer.HasExpired())
    {
      sjsu::LogWarning("Server timed out! Reconnecting...");
      Initialize();
    }
  }

  /// Connects to the URL provided in member function
  void ConnectToServer()
  {
    try
    {
      sjsu::LogInfo("Connecting to %s...", url_.data());
      socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, url_, kPort,
                      kDefaultTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error connecting to server!");
      throw e;
    }
  }

  /// Verifies that the Wi-Fi module is still connected to the network
  /// @return true if the module is still connected to the internet
  bool IsConnected()
  {
    return wifi_.IsConnected();  // TODO: Always returns false
  };

 private:
  /// Sends an HTTP request to the connected server
  void WriteToServer()
  {
    try
    {
      std::span write_payload(
          reinterpret_cast<const uint8_t *>(request_.data()), request_.size());
      socket_.Write(write_payload, kDefaultTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error writing to server!");
      throw e;
    }
  }

  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;
  std::string request_;
  std::string url_                               = "192.168.113.50";
  std::string kErrorResponse                     = "ERROR";
  const uint16_t kPort                           = 5000;
  const char * kSsid                             = "kingdom2";
  const char * kPassword                         = "22039622";
  const std::chrono::nanoseconds kDefaultTimeout = 10s;
};
}  // namespace sjsu::common
