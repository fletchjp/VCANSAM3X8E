
// VLCB4IN4OUT
// Version for use with Arduino DUE


/*
   Copyright (C) 2023 Martin Da Costa, (C) 2024 John Fletcher
  //  This file is part of VLCB-Arduino project on https://github.com/SvenRosvall/VLCB-Arduino
  //  Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  //  The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0

*/

/*
      3rd party libraries needed for compilation:

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      due_can     -- library to support the DUE CAN controller implementation
      can_common  -- library used by due_can
      CBUSSwitch  -- library access required by CBUS and CBUS Config
      CBUSLED     -- library access required by CBUS and CBUS Config
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map:

//////////////////////////////////////////////////////////////////////////

#define DEBUG 0  // set to 0 for no serial debug

#if DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// 3rd party libraries
#include <Streaming.h>
#include <Bounce2.h>

////////////////////////////////////////////////////////////////////////////
// VLCB library header files
// Uncomment this to use external EEPROM
//#define USE_EXTERNAL_EEPROM
////////////////////////////////////////////////////////////////////////////
#include <Controller.h>                // Controller class
#include "VCANSAM3X8E.h"               // CAN controller
#ifdef USE_EXTERNAL_EEPROM
#define EEPROM_I2C_ADDR 0x50
#include <Wire.h>
#include <EepromExternalStorage.h>
#else
//#include <CreateDefaultStorageForPlatform.h>
#include <DueEepromEmulationStorage.h>
#endif
#include <Configuration.h>             // module configuration
#include <Parameters.h>               // VLCB parameters
#include <vlcbdefs.hpp>               // VLCB constants
#include <LEDUserInterface.h>
#include "MinimumNodeService.h"
#include "CanService.h"
#include "NodeVariableService.h"
#include "EventConsumerService.h"
#include "EventProducerService.h"
#include "ConsumeOwnEventsService.h"
#include "EventTeachingService.h"
#include "SerialUserInterface.h"

// Module library header files
#include "LEDControl.h"

// constants
const byte VER_MAJ = 1;               // code major version
const char VER_MIN = 'a';             // code minor version
const byte VER_BETA = 0;              // code beta sub-version
const byte MANUFACTURER = MANU_DEV;   // Module Manufacturer set to Development
const byte MODULE_ID = 82;            // CBUS module type

const byte LED_GRN = 14;         // VLCB green Unitialised LED pin
const byte LED_YLW = 15;         // VLCB yellow Normal LED pin
const byte SWITCH0 = 13;         // VLCB push button switch pin

// Controller objects
#ifdef USE_EXTERNAL_EEPROM
VLCB::EepromExternalStorage externalStorage(EEPROM_I2C_ADDR, &Wire1);
VLCB::Configuration modconfig(&externalStorage);  // configuration object
#else
VLCB::DueEepromEmulationStorage dueStorage;  // DUE simulated EEPROM
VLCB::Configuration modconfig(&dueStorage);  // configuration object
#endif
VLCB::VCANSAM3X8E vcanSam3x8e;  // CAN transport object
VLCB::LEDUserInterface ledUserInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface(&vcanSam3x8e);
VLCB::MinimumNodeService mnService;
VLCB::CanService canService(&vcanSam3x8e);
VLCB::NodeVariableService nvService;
VLCB::ConsumeOwnEventsService coeService;
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;
VLCB::Controller controller(&modconfig,
                            {&mnService, &ledUserInterface, &serialUserInterface, &canService, &nvService, &ecService, &epService, &etService, &coeService}); // Controller object

// module name, must be 7 characters, space padded.
unsigned char mname[7] = { '4', 'I', 'N', '4', 'O', 'U', 'T' };

// Module objects
const byte LED[] = { 22, 26, 27, 28 };     // LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = { 18, 19, 20, 21 };  // Module Switch takes input to 0V.

const bool active = 0;  // 0 is for active low LED drive. 1 is for active high

const byte NUM_LEDS = sizeof(LED) / sizeof(LED[0]);
const byte NUM_SWITCHES = sizeof(SWITCH) / sizeof(SWITCH[0]);

// module objects
Bounce moduleSwitch[NUM_SWITCHES];  //  switch as input
LEDControl moduleLED[NUM_LEDS];     //  LED as output
byte switchState[NUM_SWITCHES];

// forward function declarations
void eventhandler(byte, const VLCB::VlcbMessage *);
void printConfig();
void processSwitches();

//
///  setup VLCB - runs once at power on called from setup()
//
void setupVLCB() {
  // set config layout parameters
  modconfig.EE_NVS_START = 10;
  modconfig.EE_NUM_NVS = NUM_SWITCHES;
  modconfig.EE_EVENTS_START = 50;
  modconfig.EE_MAX_EVENTS = 64;
  modconfig.EE_PRODUCED_EVENTS = NUM_SWITCHES;
  modconfig.EE_NUM_EVS = 1 + NUM_LEDS;  

  // initialise and load configuration
  controller.begin();

  Serial << F("> mode = ") << ((modconfig.currentMode) ? "Normal" : "Uninitialised") << F(", CANID = ") << modconfig.CANID;
  Serial << F(", NN = ") << modconfig.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  VLCB::Parameters params(modconfig);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setManufacturer(MANUFACTURER);
  params.setModuleId(MODULE_ID);  

  // assign to Controller
  controller.setParams(params.getParams());
  controller.setName(mname);

  // module reset - if switch is depressed at startup
  if (ledUserInterface.isButtonPressed())
  {
    Serial << F("> switch was pressed at startup") << endl;
    modconfig.resetModule();
  }

  // register our VLCB event handler, to receive event messages of learned events
  ecService.setEventHandler(eventhandler);

  // set Controller LEDs to indicate mode
  controller.indicateMode(modconfig.currentMode);

  vcanSam3x8e.setControllerInstance(0);  // only actually required for instance 1, instance 0 is the default

  if (!vcanSam3x8e.begin()) {
    Serial << F("> error starting VLCB") << endl;
  } else {
    Serial << F("> VLCB started") << endl;
  }
}

//
///  setup Module - runs once at power on called from setup()
//

void setupModule()
{
  unsigned int nodeNum = modconfig.nodeNum;
  // configure the module switches, active low
  for (byte i = 0; i < NUM_SWITCHES; i++)
  {
    moduleSwitch[i].attach(SWITCH[i], INPUT_PULLUP);
    moduleSwitch[i].interval(5);
    switchState[i] = false;
  }

  // configure the module LEDs
  for (byte i = 0; i < NUM_LEDS; i++)
  {
    moduleLED[i].setPin(LED[i], active);  //Second arguement sets 0 = active low, 1 = active high. Default if no second arguement is active high.
    moduleLED[i].off();
  }

  Serial << "> Module has " << NUM_LEDS << " LEDs and " << NUM_SWITCHES << " switches." << endl;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial << endl << endl << F("> ** VLCB 4 in 4 out Pico single core ** ") << __FILE__ << endl;

  setupVLCB();
  setupModule();

  // end of setup
  Serial << F("> ready") << endl << endl;
}

void loop()
{

  // do VLCB message, switch and LED processing
  controller.process();

  // Run the LED code
  for (byte i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();

  // end of loop()
}

void processSwitches(void)
{
  for (byte i = 0; i < NUM_SWITCHES; i++)
  {
    moduleSwitch[i].update();
    if (moduleSwitch[i].changed())
    {
      byte nv = i + 1;
      byte nvval = modconfig.readNV(nv);
      bool state;
      byte swNum = i + 1;

      DEBUG_PRINT(F("sk> Button ") << i << F(" state change detected. NV Value = ") << nvval);

      switch (nvval)
      {
        case 1:
          // ON and OFF
          state = (moduleSwitch[i].fell());
          DEBUG_PRINT(F("sk> Button ") << i << (moduleSwitch[i].fell() ? F(" pressed, send state: ") : F(" released, send state: ")) << state);
          epService.sendEvent(state, swNum);
          break;

        case 2:
          // Only ON
          if (moduleSwitch[i].fell()) 
          {
            state = true;
            DEBUG_PRINT(F("sk> Button ") << i << F(" pressed, send state: ") << state);
            epService.sendEvent(state, swNum);
          }
          break;

        case 3:
          // Only OFF
          if (moduleSwitch[i].fell())
          {
            state = false;
            DEBUG_PRINT(F("sk> Button ") << i << F(" pressed, send state: ") << state);
            epService.sendEvent(state, swNum);
          }
          break;

        case 4:
          // Toggle button
          if (moduleSwitch[i].fell())
          {
            switchState[i] = !switchState[i];
            state = (switchState[i]);
            DEBUG_PRINT(F("sk> Button ") << i << (moduleSwitch[i].fell() ? F(" pressed, send state: ") : F(" released, send state: ")) << state);
            epService.sendEvent(state, swNum);
          }
          break;

        default:
          DEBUG_PRINT(F("sk> Button ") << i << F(" do nothing."));
          break;
      }
    }
  }
}

//
/// called from the VLCB library when a learned event is received
//
void eventhandler(byte index, const VLCB::VlcbMessage *msg)
{
  byte opc = msg->data[0];

  DEBUG_PRINT(F("sk> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));

  unsigned int node_number = (msg->data[1] << 8) + msg->data[2];
  unsigned int event_number = (msg->data[3] << 8) + msg->data[4];
  DEBUG_PRINT(F("sk> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(F("sk> op_code = ") << _HEX(opc));

  switch (opc)
  {
    case OPC_ACON:
    case OPC_ASON:
      DEBUG_PRINT(F("sk> case is opCode ON"));
      for (byte i = 0; i < NUM_LEDS; i++)
      {
        byte ev = i + 2;
        byte evval = modconfig.getEventEVval(index, ev);
        //DEBUG_PRINT(F("sk> EV = ") << ev << (" Value = ") << evval);

        switch (evval) 
        {
          case 1:
            moduleLED[i].on();
            break;

          case 2:
            moduleLED[i].flash(500);
            break;

          case 3:
            moduleLED[i].flash(250);
            break;

          default:
            break;
        }
      }
      break;

    case OPC_ACOF:
    case OPC_ASOF:
      DEBUG_PRINT(F("sk> case is opCode OFF"));
      for (byte i = 0; i < NUM_LEDS; i++)
      {
        byte ev = i + 2;
        byte evval = modconfig.getEventEVval(index, ev);

        if (evval > 0)
        {
          moduleLED[i].off();
        }
      }
      break;
  }
}

void printConfig(void) {
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Martin Da Costa (MERG M6237) 2024") << endl;
}
