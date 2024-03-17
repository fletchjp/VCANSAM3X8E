/////////////////////////////////////////////////////////////////////////////
// VLCB_DUE_long_message_example.ino
/////////////////////////////////////////////////////////////////////////////
// Version using external EEPROM using I2C over Wire1.
/////////////////////////////////////////////////////////////////////////////
//  Copyright (C) Sven Rosvall (sven@rosvall.ie)
//  This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
//  Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//  The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
*/

// 3rd party libraries
#include <Streaming.h>

////////////////////////////////////////////////////////////////////////////
// VLCB library header files
// Uncomment this to use external EEPROM
//#define USE_EXTERNAL_EEPROM
////////////////////////////////////////////////////////////////////////////
#include <Controller.h>                // Controller class
#include <VCANSAM3X8E.h>               // CAN controller
#ifdef USE_EXTERNAL_EEPROM
#define EEPROM_I2C_ADDR 0x50
#include <Wire.h>
#include <EepromExternalStorage.h>
#else
//#include <CreateDefaultStorageForPlatform.h>
#include <DueEepromEmulationStorage.h>
#endif
#include <Configuration.h>             // module configuration
#include <Parameters.h>                // VLCB parameters
#include <vlcbdefs.hpp>                // VLCB constants
#include <LEDUserInterface.h>
#include "MinimumNodeService.h"
#include <LongMessageService.h>
#include "CanService.h"
#include "NodeVariableService.h"
#include "EventConsumerService.h"
#include "EventProducerService.h"
#include "EventTeachingService.h"
#include "SerialUserInterface.h"

// constants
const byte VER_MAJ = 1;             // code major version
const char VER_MIN = 'a';           // code minor version
const byte VER_BETA = 0;            // code beta sub-version
const byte MODULE_ID = 99;          // VLCB module type

const byte LED_GRN = 4;             // VLCB green Unitialised LED pin
const byte LED_YLW = 7;             // VLCB yellow Normal LED pin
const byte SWITCH0 = 8;             // VLCB push button switch pin

#define DEBUG 1         // set to 0 for no serial debug

#if DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// Controller objects
// Controller objects
#ifdef USE_EXTERNAL_EEPROM
VLCB::EepromExternalStorage externalStorage(EEPROM_I2C_ADDR, &Wire1);
VLCB::Configuration modconfig(&externalStorage);  // configuration object
#else
VLCB::DueEepromEmulationStorage dueStorage;  // DUE simulated EEPROM
VLCB::Configuration modconfig(&dueStorage);  // configuration object
#endif
VLCB::VCANSAM3X8E vcanSam3x8e;  // CAN transport object
//VLCB::LEDUserInterface userInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface(&vcanSam3x8e);
VLCB::MinimumNodeService mnService;
VLCB::CanService canService(&vcanSam3x8e);
VLCB::NodeVariableService nvService;
VLCB::LongMessageService lmsg;        // Controller RFC0005 long message object
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;
VLCB::Controller controller(&modconfig,
                            {&mnService, &serialUserInterface, &canService, &nvService, &lmsg, &ecService, &epService, &etService}); // Controller object

// module name, must be 7 characters, space padded.
unsigned char mname[7] = { 'L', 'M', 'S', 'G', 'E', 'X', ' ' };

// forward function declarations
void eventhandler(byte, const VLCB::VlcbMessage *);
void processSerialInput();
void printConfig();
void longmessagehandler(void *, const unsigned int, const byte, const byte);

// long message variables
byte streams[] = {1, 2, 3};         // streams to subscribe to
char lmsg_out[32], lmsg_in[32];     // message buffers

//
/// setup VLCB - runs once at power on from setup()
//
void setupVLCB()
{
  // set config layout parameters
  modconfig.EE_NVS_START = 10;
  modconfig.EE_NUM_NVS = 10;
  modconfig.EE_EVENTS_START = 50;
  modconfig.EE_MAX_EVENTS = 32;
  modconfig.EE_PRODUCED_EVENTS = 1;
  modconfig.EE_NUM_EVS = 1;

  // initialise and load configuration
  controller.begin();

  Serial << F("> mode = ") << ((modconfig.currentMode) ? "Normal" : "Uninitialised") << F(", CANID = ") << modconfig.CANID;
  Serial << F(", NN = ") << modconfig.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  VLCB::Parameters params(modconfig);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setManufacturer(MANU_DEV);
  params.setModuleId(MODULE_ID);

  // assign to Controller
  controller.setParams(params.getParams());
  controller.setName(mname);

  // module reset - if switch is depressed at startup
  //if (userInterface.isButtonPressed())
  //{
  //  Serial << F("> switch was pressed at startup") << endl;
  //  modconfig.resetModule();
  //}

  // register our VLCB event handler, to receive event messages of learned events
  ecService.setEventHandler(eventhandler);

  // subscribe to long message streams and register our handler function
  lmsg.subscribe(streams, sizeof(streams) / sizeof(byte), lmsg_in, 32, longmessagehandler);

  // set Controller LEDs to indicate the current mode
  controller.indicateMode(modconfig.currentMode);

  vcanSam3x8e.setControllerInstance(0);  // only actually required for instance 1, instance 0 is the default
  if (!vcanSam3x8e.begin()) {
    DEBUG_PRINT("> error starting VLCB");
  } else {
    DEBUG_PRINT("> VLCB started");
  }

}

//
/// setup - runs once at power on
//
void setup()
{
  Serial.begin (115200);
  Serial << endl << endl << F("> ** VLCB Arduino long message example module ** ") << __FILE__ << endl;

  setupVLCB();

  // end of setup
  Serial << F("> ready") << endl << endl;
}

//
/// loop - runs forever
//
void loop()
{
  //
  /// do VLCB message, switch and LED processing
  //
  controller.process();

  //
  /// do RFC0005 Controller long message processing
  //
  if (!lmsg.process()) {
    Serial << F("> error in long message processing") << endl;
  }

  // bottom of loop()
}

//
/// user-defined event processing function
/// called from the VLCB library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte index, const VLCB::VlcbMessage *msg)
{
  // as an example, display the opcode and the first EV of this event, which is ev2 as ev1 defines produced event

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;
  Serial << F("> EV1 = ") << modconfig.getEventEVval(index, 2) << endl;
}

//
/// long message receive handler function
/// called once the user buffer is full, or the message has been completely received
//
void longmessagehandler(void *fragment, const unsigned int fragment_len, const byte stream_id, const byte status)
{
  // display the message
  // this will be the complete message if shorter than the provided buffer, or the final fragment if longer

  if (status == VLCB::LONG_MESSAGE_COMPLETE) {
    Serial << F("> received long message, stream = ") << stream_id << F(", len = ") << fragment_len << F(", msg = |") << (char *) fragment << F("|") << endl;
  }
}

//
/// print code version config details and copyright notice
//
void printConfig()
{
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2019") << endl;
}

