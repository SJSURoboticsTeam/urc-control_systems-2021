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
  Esp_v2(std::string ssid,
         std::string password,
         std::string url,
         std::string endpoint,
         int port_number)
      : esp_(sjsu::lpc40xx::GetUart<3>()),
        wifi_(esp_.GetWiFi()),
        socket_(esp_.GetInternetSocket()),
        ssid_(ssid),
        password_(password),
        url_(url),
        endpoint_(endpoint),
        port_number_(port_number){};

  void Initialize()
  {
    sjsu::LogInfo("Initializing esp module...");
    esp_.Initialize();
    Connect();
  };

  void Connect() override
  {
    ConnectToWifi();
    ConnectToWebServer();
  }

  void Disconnect() override
  {
    wifi_.DisconnectFromAccessPoint();
  }

  void SendMessage(std::string parameters) override
  {
    CreateMessage(parameters);
    SendHTTPRequest();
    ParseResponse();
  }

  std::string GetMessageResponse() override
  {
    return response_body_;
  }

  void ReconnectIfServerTimedOut(sjsu::TimeoutTimer & serverTimer)
  {
    if (serverTimer.HasExpired())
    {
      sjsu::LogWarning("Server timed out! Reconnecting...");
      Connect();
    }
  }

 private:
  void CreateMessage(std::string parameters)
  {
    char request_message[500];
    snprintf(request_message, 500, GET_request_format, endpoint_.c_str(),
             parameters.c_str(), url_.c_str());
    request_message_ = request_message;
  }

  void ParseResponse()
  {
    std::fill(raw_.begin(), raw_.end(), 0);
    size_t read_back = socket_.Read(raw_, kDefaultTimeout);
    std::string response(reinterpret_cast<char *>(raw_.data()), read_back);
    response_header_ = response.substr(0, response.find("\r\n\r\n"));
  }

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

  void ConnectToWebServer()
  {
    try
    {
      sjsu::LogInfo("Connecting to %s...", url_.data());
      socket_.Connect(sjsu::InternetSocket::Protocol::kTCP, url_,
                      static_cast<uint16_t>(port_number_), kDefaultTimeout);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error connecting to server!");
    }
  }

  void SendHTTPRequest()
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

  sjsu::Esp8266 esp_;
  sjsu::WiFi & wifi_;
  sjsu::InternetSocket & socket_;

  std::string ssid_                    = "";
  std::string password_                = "";
  std::string url_                     = "";
  std::string endpoint_                = "";
  std::string request_message_         = "";
  std::string response_header_         = "";
  std::string response_body_           = "";
  int port_number_                     = 5000;
  std::chrono::seconds kDefaultTimeout = 10s;
  std::array<uint8_t, 1024 * 2> raw_   = {};
};
}  // namespace sjsu::common
