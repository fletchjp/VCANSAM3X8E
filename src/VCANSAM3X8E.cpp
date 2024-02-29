// Copyright (C) Sven Rosvall (sven@rosvall.ie) (C( John Fletcher (john@bunbury28.plus.com)
// This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
// Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0/

// Arduino libraries
#include <SPI.h>

// 3rd party libraries
#include <Streaming.h>

// VLCB SAM3X8E library
#include "CAN2515.h"
#include "VCANSAM3X8E.h"


namespace VLCB
{


//
/// format and display CAN message
//

void format_message(CANFrame *msg)
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

  return;
}

//void something() { }

//
/// constructor
//
VCANSAM3X8E::VCANSAM3X8E()
  : _num_rx_buffers(NUM_RX_BUFFS), _num_tx_buffers(NUM_TX_BUFFS), _csPin(MCP2515_CS), _intPin(MCP2515_INT), _osc_freq(OSCFREQ)
{
  _instance = 0;
  _can = &Can0;
}

VCANSAM3X8E::~VCANSAM3X8E()
{}

//
/// initialise the CAN controller and buffers, and attach the ISR
//

bool VCANSAM3X8E::begin(bool poll, SPIClass spi)
{

  uint32_t init_ret;
  int init_watch;

  _numMsgsSent = 0;
  _numMsgsRcvd = 0;

  // init CAN instance
  init_ret = _can->begin(CAN_BPS_125K, 255);

  if (!init_ret) {
    Serial << "> CAN error from begin(), ret = " << init_ret << endl;
    return false;
  }

  // set filter to permissive
  init_watch = _can->watchFor();

  if (init_watch == -1) {  // Test changed by JPF
    Serial << "> CAN error from watchFor(), ret = " << init_watch << endl;
    return false;
  }

  Serial << "> CAN controller instance = " << _instance << " initialised successfully" << endl;
  return true;
}

bool VCANSAM3X8E::available(void)
{

  return _can->available();
}

CANFrame VCANSAM3X8E::getNextCanFrame(void)
{

  uint32_t ret;
  CAN_FRAME cf;
  CANFrame message;

  ret = _can->read(cf);

  if (ret != 1 /*CAN_MAILBOX_TRANSFER_OK */) {
    Serial << "> CAN read did not return CAN_MAILBOX_TRANSFER_OK, instance = " << _instance << ", ret = " << ret << endl;
    Serial << "> received CAN message has length = " << cf.length << endl;
  } else {
    Serial << "> received CAN message ok, instance = " << _instance << ", length = " << cf.length;
    Serial << ", rtr = " << cf.rtr << endl;
  }

  message.id = cf.id;
  message.len = cf.length;
  message.rtr = cf.rtr;
  message.ext = cf.extended;

  memcpy(message.data, cf.data.byte, cf.length);

  ++_numMsgsRcvd;

  format_message(&message);
  return message;
}

//
/// send a CBUS message
//

bool VCANSAM3X8E::sendCanFrame(CANFrame *msg)  // bool rtr, bool ext, byte priority)
{
  // note default arguments put here as a fix are not needed.
  //bool rtr = false; bool ext = false; byte priority = DEFAULT_PRIORITY;
  Serial << F("CANSAM3X8E sendCanFrame id=") << (msg->id & 0x7F) << " len=" << msg->len << " rtr=" << msg->rtr;
  if (msg->len > 0)
    Serial << " op=" << _HEX(msg->data[0]);
  Serial << endl;

  bool ret;
  CAN_FRAME cf;  // library-specific CAN message structure

  //makeHeader(msg, priority);          // set the CBUS header - CANID and priority bits
  format_message(msg);

  cf.id = msg->id;
  cf.length = msg->len;
  cf.rtr = msg->rtr;
  cf.extended = msg->ext;

  memcpy(cf.data.bytes, msg->data, msg->len);

  ret = _can->sendFrame(cf);

  if (!ret) {
    Serial << "> error sending CAN frame, instance = " << _instance << ", ret = " << ret << endl;
  }
  ++_numMsgsSent;

  // Simple workaround for sending many messages. Let the underlying hardware some time to send this message before next.
  // TODO: Replace this with monitoring of the transmit queue.
  delay(1);

  return ret;
}

//
/// display the CAN bus status instrumentation
//

void VCANSAM3X8E::printStatus(void)
{

  return;
}

//
/// reset the CAN driver
//

void VCANSAM3X8E::reset(void)
{
}

//
/// set the TX and RX pins
//

void VCANSAM3X8E::setPins(byte txPin, byte rxPin)
{

  return;
}

//
/// set the depth of the TX and RX queues
//

void VCANSAM3X8E::setNumBuffers(byte num)
{

  return;
}

//

//
/// set the CAN controller peripheral instance, there are two, default is zero
//

void VCANSAM3X8E::setControllerInstance(byte instance)
{

  Serial << "> setting CAN controller instance to " << instance << endl;
  _instance = instance;
  _can = (_instance == 0) ? &Can0 : &Can1;
}

// End of namespace VLCB
}
