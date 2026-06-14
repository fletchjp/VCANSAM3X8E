// Copyright (C) Sven Rosvall (sven@rosvall.ie) (C) John Fletcher (john@bunbury28.plus.com)
// This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
// Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0/

#pragma once

#include <Controller.h>
#include <CanTransport.h>
#include <vlcbdefs.hpp>
#include <SPI.h>
#include <due_can.h>

namespace VLCB
{

class VCANSAM3X8E : public CanTransport
{
public:
  VCANSAM3X8E();
  virtual ~VCANSAM3X8E();

  bool begin(bool poll = false, SPIClass spi = SPI);
  bool available() override;
  CANFrame getNextCanFrame() override;
  bool sendCanFrame(CANFrame *msg) override;
  void reset() override;

  void printStatus(void);
  void setControllerInstance(byte instance = 0);
  void setDebug(bool new_state);

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

  virtual byte getHardwareType() override
  {
    return CAN_HW_SAM3X8E;
  }

  virtual unsigned int receiveBufferUsage() override
  {
    return _can->available();
  }

  virtual unsigned int transmitBufferUsage() override
  {
    return 0;
  }

  virtual unsigned int receiveBufferPeak() override
  {
    return _hwmRx;
  }

  virtual unsigned int transmitBufferPeak() override
  {
    return 0;
  }

private:
  unsigned int _numMsgsSent = 0;
  unsigned int _numMsgsRcvd = 0;
  unsigned int _hwmRx = 0;
  byte _instance = 0;
  CANRaw *_can = nullptr;
  bool _debug_on = false;
};

}

