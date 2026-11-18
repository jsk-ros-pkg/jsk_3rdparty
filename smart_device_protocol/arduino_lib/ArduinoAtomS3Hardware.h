#include <Arduino.h>  // Arduino 1.0

#define SERIAL_CLASS HWCDC

class ArduinoHardware
{
public:
  ArduinoHardware(SERIAL_CLASS* io, long baud = 57600)
  {
    iostream = io;
    baud_ = baud;
  }
  ArduinoHardware()
  {
    iostream = &Serial;
    baud_ = 57600;
  }
  ArduinoHardware(ArduinoHardware& h)
  {
    this->iostream = h.iostream;
    this->baud_ = h.baud_;
  }

  void setBaud(long baud)
  {
    this->baud_ = baud;
  }

  int getBaud()
  {
    return baud_;
  }

  void init()
  {
    iostream->begin(baud_);
  }

  int read()
  {
    return iostream->read();
  };
  void write(uint8_t* data, int length)
  {
    for (int i = 0; i < length; i++)
      iostream->write(data[i]);
  }

  unsigned long time()
  {
    return millis();
  }

protected:
  SERIAL_CLASS* iostream;
  long baud_;
};

