// Copyright (C) Sven Rosvall (sven@rosvall.ie) (C) John Fletcher (john@bunbury28.plus.com)
// This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
// Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0/

#include <SPI.h>
#include <Streaming.h>
#include "VCANSAM3X8E.h"

namespace VLCB
{

namespace
{

void formatMessage(CANFrame *msg)
{
  char mbuff[80], dbuff[8];

  sprintf(mbuff, "[%03ld] [%d] [", (msg->id & 0x7f), msg->len);

  for (byte i = 0; i < msg->len; i++) {
    sprintf(dbuff, " %02x", msg->data[i]);
    strcat(mbuff, dbuff);
  }

  strcat(mbuff, " ]");
  Serial << "> " << mbuff;
  if (msg->rtr) Serial << " rtr = " << msg->rtr;
  Serial << endl;
}

}

VCANSAM3X8E::VCANSAM3X8E()
{
  _can = &Can0;
}

VCANSAM3X8E::~VCANSAM3X8E()
{
}

bool VCANSAM3X8E::begin(bool /*poll*/, SPIClass /*spi*/)
{
  _numMsgsSent = 0;
  _numMsgsRcvd = 0;

  bool init_ret = _can->begin(CAN_BPS_125K, 255);

  if (!init_ret) {
    if (_debug_on) Serial << "> CAN error from begin(), ret = " << init_ret << endl;
    return false;
  }

  int init_watch = _can->watchFor();

  if (init_watch == -1) {
    if (_debug_on) Serial << "> CAN error from watchFor(), ret = " << init_watch << endl;
    return false;
  }

  if (_debug_on) Serial << "> CAN controller instance = " << _instance << " initialised successfully" << endl;
  return true;
}

bool VCANSAM3X8E::available(void)
{
  return _can->available();
}

CANFrame VCANSAM3X8E::getNextCanFrame(void)
{
  CAN_FRAME cf;
  CANFrame message;

  uint32_t ret = _can->read(cf);

  if (ret != 1) {
    if (_debug_on) {
      Serial << "> CAN read error, instance = " << _instance << ", ret = " << ret << endl;
    }
  }

  message.id = cf.id;
  message.len = cf.length;
  message.rtr = cf.rtr;
  message.ext = cf.extended;

  memcpy(message.data, cf.data.byte, cf.length);

  ++_numMsgsRcvd;

  if (_debug_on) {
    formatMessage(&message);
  }
  return message;
}

bool VCANSAM3X8E::sendCanFrame(CANFrame *msg)
{
  if (_debug_on) {
    Serial << "sendCanFrame id=" << (msg->id & 0x7F) << " len=" << msg->len << " rtr=" << msg->rtr;
    if (msg->len > 0)
      Serial << " op=" << _HEX(msg->data[0]);
    Serial << endl;
    formatMessage(msg);
  }

  CAN_FRAME cf;

  cf.id = msg->id;
  cf.length = msg->len;
  cf.rtr = msg->rtr;
  cf.extended = msg->ext;

  memcpy(cf.data.bytes, msg->data, msg->len);

  uint32_t start = micros();
  bool ret;
  do {
    ret = _can->sendFrame(cf);
    if (ret) break;
  } while (micros() - start < 1000);

  if (ret) {
    ++_numMsgsSent;
  } else {
    if (_debug_on) {
      Serial << "> error sending CAN frame, instance = " << _instance << endl;
    }
  }

  return ret;
}

void VCANSAM3X8E::printStatus(void)
{
  if (_debug_on) {
    Serial << "> VLCB status: messages received = " << _numMsgsRcvd
           << ", sent = " << _numMsgsSent
           << ", rx errors = " << _can->get_rx_error_cnt()
           << ", tx errors = " << _can->get_tx_error_cnt()
           << ", status = " << _can->get_status() << endl;
  }
}

void VCANSAM3X8E::reset(void)
{
  _can->begin(CAN_BPS_125K, 255);
  _can->watchFor();
  _numMsgsSent = 0;
  _numMsgsRcvd = 0;

  if (_debug_on) Serial << "> CAN controller reset" << endl;
}

void VCANSAM3X8E::setControllerInstance(byte instance)
{
  _instance = instance;
  _can = (_instance == 0) ? &Can0 : &Can1;

  if (_debug_on) Serial << "> setting CAN controller instance to " << instance << endl;
}

void VCANSAM3X8E::setDebug(bool new_state)
{
  _debug_on = new_state;
}

}
