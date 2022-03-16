class CommunicationInterface
{
 public:
  virtual void Connect()                   = 0;
  virtual void Disconnect()                = 0;
  virtual bool IsConnected()               = 0;
  virtual void SendMessage(std::string)    = 0;
  virtual std::string GetMessageResponse() = 0;
};
