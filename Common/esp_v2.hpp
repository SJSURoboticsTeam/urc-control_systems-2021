#pragma once

#include <string_view>
#include <algorithm>

#include "utility/log.hpp"
#include "peripherals/lpc40xx/uart.hpp"
#include "devices/communication/esp8266.hpp"
#include "Interface/communication_interface.hpp"

namespace sjsu::common
{
const char GET_request_format[] = "GET /%s%s HTTP/1.1\r\nHost: %s\r\n\r\n";

class Esp_v2 : public CommunicationInterface
{
 public:
  Esp_v2(string ssid, string password, string url, string port_number)
      : ssid_(ssid),
        password_(password),
        url_(url),
        port_number_(port_number){};

  void Initialize()
  {
    sjsu::LogInfo("Initializing esp module...");
    esp_.Initialize();
    Connect();
  };

  void Connect()
  {
    ConnectToWifi();
    ConnectToServer();
  }

  void Disconnect()
  {
    wifi_.DisconnectFromAccessPoint();
  }

  void CreateRequest(string endpoint, string parameters)
  {
    char request_message[500];
    snprintf(request_message, 500, GET_request_format, endpoint, parameters,
             url_);
    request_message_ = request_message;
  }

  string GetResponse()
  {
    return response_body_;
  }

  void SendRequest()
  {
    WriteToServer();
  }

 private:
  void ParseResponse()
  {
    std::fill(raw_.begin(), raw_.end(), 0);
    size_t read_back = socket_.Read(raw_, kDefaultTimeout);
    std::string response(reinterpret_cast<char *>(raw_.data()), read_back);
    response_header_ = response.substr(response.find)
  }

  /// Sends a GET request to the hardcoded URL
  /// @param endpoint i.e. /endpoint?example=parameter
  /// @return the response body of the GET request
  std::string GET(std::string endpoint)
  {
    try
    {
      try
      {
        response = response.substr(response.find("\r\n\r\n{"));
        printf("Response Body:%s", response.c_str());
        return response;
      }
      catch (const std::exception & e)
      {
        sjsu::LogError("Error parsing response from server!");
      }
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error reading response from server!");
    }
    return "Error";
  };

  /// Attempts to connect to the local WiFi network
  void ConnectToWifi()
  {
    while (true)
    {
      sjsu::LogInfo("Attempting to connect to %s...", ssid_);
      if (wifi_.ConnectToAccessPoint(ssid_, password_, kDefaultTimeout))
      {
        break;
      }
      sjsu::LogWarning("Connecting...", ssid_);
    }
  }

  void IsServerExpired(sjsu::TimeoutTimer & serverTimer)
  {
    if (serverTimer.HasExpired())
    {
      sjsu::LogWarning("Server timed out! Reconnecting...");
      Connect();
    }
  }

  /// Connects to the URL provided in member function
  void ConnectToServer()
  {
    try
    {
      sjsu::LogInfo("Connecting to %s...", url_.data());
      socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, url_, port_number_,
                      kDefaultTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error connecting to server!");
    }
  }

  bool IsConnected()
  {
    return wifi_.IsConnected();  // TODO: Always returns false
  };

  /// Sends an HTTP request to the connected server
  void WriteToServer()
  {
    try
    {
      std::span write_payload(
          reinterpret_cast<const uint8_t *>(request_message_.data()),
          request_message_.size());
      socket_.Write(write_payload, kDefaultTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error writing to server!");
    }
  }

  std::array<uint8_t, 1024 * 2> raw_;

  int port_number_                     = 5000;
  std::string url_                     = "";
  std::string ssid_                    = "";
  std::string password_                = "";
  std::string request_message_         = "";
  std::string response_header_         = "";
  std::string response_body_           = "";
  std::chrono::seconds kDefaultTimeout = 10s;
  sjsu::Esp8266 esp_                   = sjsu::lpc40xx::GetUart<3>();
  sjsu::WiFi & wifi_                   = esp_.GetWiFi();
  sjsu::InternetSocket & socket_       = esp_.GetInternetSocket();
};
}  // namespace sjsu::common
