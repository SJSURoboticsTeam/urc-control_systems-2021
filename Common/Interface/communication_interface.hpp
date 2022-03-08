class CommunicationInterface
{
 public:
  virtual void Connect()           = 0;
  virtual void Disconnect()        = 0;
  virtual bool IsConnected()       = 0;
  virtual void SendRequest(string) = 0;
  virtual string GetRequest()      = 0;
};
