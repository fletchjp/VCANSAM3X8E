// Copyright (C) Sven Rosvall (sven@rosvall.ie) (C) John Fletcher (john@bunbury28.plus.com)
// This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
// Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0/

#pragma once

// header files

#include <Controller.h>
#include <CanTransport.h>
#include <SPI.h>
#include <due_can.h>  // Due CAN library header file

namespace VLCB
{

// constants
// These are getting defined somewhere else.....
//static const byte MCP2515_CS = 10;                          // SPI chip select pin
//static const byte MCP2515_INT = 2;                          // interrupt pin
//static const byte NUM_RX_BUFFS = 4;                         // default value
//static const byte NUM_TX_BUFFS = 2;                         // default value
//static const uint32_t CANBITRATE = 125000UL;                // 125Kb/s - fixed for VLCB
//static const uint32_t OSCFREQ = 16000000UL;                 // crystal frequency default

//
/// an implementation of the Transport interface class
/// to support the SAM3X8E CAN controllers for Arduino DUE.
//

class VCANSAM3X8E : public CanTransport
{
public:

  VCANSAM3X8E();
  virtual ~VCANSAM3X8E();

  // these methods are declared virtual in the base class and must be implemented by the derived class
#ifdef ARDUINO_ARCH_RP2040
  bool begin(bool poll = false, SPIClassRP2040 spi = SPI);  // note default args
#else
  bool begin(bool poll = false, SPIClass spi = SPI);
#endif
  bool available() override;
  CANFrame getNextCanFrame() override;
  bool sendCanFrame(CANFrame *msg) override;
  void reset() override;

  // these methods are specific to this implementation
  // they are not declared or implemented by the base CBUS class
  void setNumBuffers(byte num);
  void setPins(byte rxpin, byte rxPin);
  void printStatus(void);
  void setControllerInstance(byte instance = 0);

  virtual unsigned int receiveCounter() override
  {
    return _numMsgsRcvd;
  }
  virtual unsigned int transmitCounter() override
  {
    return _numMsgsSent;
  }
  virtual unsigned int receiveErrorCounter() override
  {
    return _can->get_rx_error_cnt();
  }
  virtual unsigned int transmitErrorCounter() override
  {
    return _can->get_tx_error_cnt();
  }
  virtual unsigned int errorStatus() override
  {
    return _can->get_status();
  }

  //ACAN2515 *canp;  // pointer to CAN object so user code can access its members

private:
  unsigned int _numMsgsSent, _numMsgsRcvd;
  unsigned long _osc_freq;
  byte _csPin, _intPin;
  byte _num_rx_buffers, _num_tx_buffers;
  bool _poll;
 
  byte _instance;
  CANRaw *_can;
};

void format_message(CANFrame *msg);


}