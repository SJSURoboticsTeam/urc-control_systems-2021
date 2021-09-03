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
  std::string GETRequest(std::string endpoint)
  {
    request_ = "GET /" + endpoint + " HTTP/1.1\r\nHost: " + url_ + "\r\n\r\n";
    WriteToServer();
    try
    {
      std::array<uint8_t, 1024 * 2> response;
      std::fill(response.begin(), response.end(), 0);
      size_t read_back = socket_.Read(response, kDefaultTimeout);
      // printf("RESPONSE:\n%s\n", response.data());
      std::string body(reinterpret_cast<char *>(response.data()), read_back);
      try
      {
        body = body.substr(body.find("\r\n\r\n"));
        body = body.substr(body.find("{"));
        printf("Response Body:\n%s\n", body.c_str());
        return body;
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
  std::string url_                               = "192.168.1.103";
  std::string kErrorResponse                     = "ERROR";
  const uint16_t kPort                           = 5000;
  const char * kSsid                             = "GarzaLine";
  const char * kPassword                         = "NRG523509";
  const std::chrono::nanoseconds kDefaultTimeout = 5s;
};
}  // namespace sjsu::common
