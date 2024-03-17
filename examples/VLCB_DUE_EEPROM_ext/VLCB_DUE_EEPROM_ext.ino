/////////////////////////////////////////////////////////////////////////////
// VLCB_DUE_EEPROM_ext
/////////////////////////////////////////////////////////////////////////////
// Version using external EEPROM using I2C over Wire1.
/////////////////////////////////////////////////////////////////////////////
// There are two CAN interfaces and I could explore using both of them.

/*
  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
*/

///////////////////////////////////////////////////////////////////////////////////
// Pin Use map DUE:
// Note that the pin layout is similar to an Arduino MEGA.
// There are a lot of other uses defined on a full pinout diagram.
// Pins used by the CAN attachment are documented.
// A lot more pins are inaccessible when it is in place.
// That includes the SDA and SCL pins for I2C using Wire.
// Instead Wire1 is used for I2C for the external EEPROM (Device 0x50).
// The pins for this (SDA1 and SCL1) are not allocated pin nos.
// They can be accessed from the DUE or via a DFRobot shield V7.1
// Both methods have been used.
/*
 Digital pin 0 RX0            
 Digital pin 1 TX0
 Digital pin 2 (PWM)               
 Digital pin 3 (PWM)
 Digital pin 4 (PWM)
 Digital pin 5 (PWM)
 Digital pin 6 (PWM)
 Digital pin 7 (PWM)
 
 Digital pin 8 (PWM)
 Digital pin 9 (PWM)
 Digital pin 10 (PWM)
 Digital pin 11 (PWM)
 Digital pin 12 (PWM)
 Digital pin 13 (PWM)
 SDA1  (Wire1) These pins don't have pin numbers.
 SCL1  (Wire1)
 
 Digital pin 14 (TX3) | CAN attachment
 Digital pin 15 (RX3) | CAN attachment
 Digital pin 16 (TX2)
 Digital pin 17 (RX2)
 Digital pin 18 (TX1)
 Digital pin 19 (RX1)
 Digital pin 20 (SDA) | CAN attachment
 Digital pin 21 (SCL) | CAN attachment
 
 Digital pin 22 | CAN attachment
 Digital pin 23 | CAN attachment
 Digital pin 24
 Digital pin 25
 Digital pin 26
 Digital pin 27
 Digital pin 28
 Digital pin 29
 Digital pin 30
 Digital pin 31
 Digital pin 32
 Digital pin 33
 Digital pin 34 (PWM 34 - 45)
 Digital pin 35 |
 Digital pin 36 |
 Digital pin 37 |
 Digital pin 38 |
 Digital pin 39 |
 Digital pin 40 |
 Digital pin 41 |
 Digital pin 42 |
 Digital pin 43 |
 Digital pin 44 |
 Digital pin 45 (PWM)
 Digital pin 46 
 Digital pin 47 
 Digital pin 48 
 Digital pin 49
 Digital pin 50
 Digital pin 51
 Digital pin 52 | CAN attachment
 Digital pin 53 | CAN attachment
 
 Digital pin 54 / Analog pin 0
 Digital pin 55 / Analog pin 1
 Digital pin 56 / Analog pin 2
 Digital pin 57 / Analog pin 3
 Digital pin 58 / Analog pin 4
 Digital pin 59 / Analog pin 5
 Digital pin 60 / Analog pin 6
 Digital pin 61 / Analog pin 7
 
 Digital pin 62 / Analog pin 8  | These pins are all
 Digital pin 63 / Analog pin 9  | used by the CAN attachment.
 Digital pin 64 / Analog pin 10 |
 Digital pin 65 / Analog pin 11 |
 Digital pin 66 / DAC0          |
 Digital pin 67 / DAC1          |
 Digital pin 68 / CANRX         |
 Digital pin 69 / CANTX         |
*/

///////////////////////////////////////////////////////////////////////////////////

// 3rd party libraries
#include <Streaming.h>
#if LCD_DISPLAY || OLED_DISPLAY
#include <Wire.h>  // Library for I2C comminications for display
#if LCD_DISPLAY
/* libraries for LCD display module */
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h >
#endif
#endif

////////////////////////////////////////////////////////////////////////////
// VLCB library header files
// Uncomment this to use external EEPROM
#define USE_EXTERNAL_EEPROM
////////////////////////////////////////////////////////////////////////////
#include <Controller.h>  // Controller class
#include <VCANSAM3X8E.h> // CAN controller
#include <Switch.h>      // pushbutton switch
#include <LED.h>         // VLCB LEDs
#ifdef USE_EXTERNAL_EEPROM
#define EEPROM_I2C_ADDR 0x50
#include <Wire.h>
#include <EepromExternalStorage.h>
#else
//#include <CreateDefaultStorageForPlatform.h>
#include <DueEepromEmulationStorage.h>
#endif
#include <Configuration.h>  // module configuration
#include <Parameters.h>     // VLCB parameters
#include <vlcbdefs.hpp>     // VLCB constants
#include <LEDUserInterface.h>
#include "MinimumNodeService.h"
#include "CanService.h"
#include "NodeVariableService.h"
#include "EventConsumerService.h"
#include "EventProducerService.h"
#include "EventTeachingService.h"
#include "SerialUserInterface.h"

// constants
const byte VER_MAJ = 1;     // code major version
const char VER_MIN = 'c';   // code minor version
const byte VER_BETA = 1;    // code beta sub-version
const byte MODULE_ID = 99;  // VLCB module type

// Controller objects
#ifdef USE_EXTERNAL_EEPROM
VLCB::EepromExternalStorage externalStorage(EEPROM_I2C_ADDR, &Wire1);
VLCB::Configuration modconfig(&externalStorage);  // configuration object
#else
VLCB::DueEepromEmulationStorage dueStorage;  // DUE simulated EEPROM
VLCB::Configuration modconfig(&dueStorage);  // configuration object
#endif
VLCB::VCANSAM3X8E vcanSam3x8e;  // CAN transport object
//VLCB::LEDUserInterface ledUserInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface(&vcanSam3x8e);
//VLCB::CombinedUserInterface combinedUserInterface(&ledUserInterface, &serialUserInterface);
VLCB::MinimumNodeService mnService;
VLCB::CanService canService(&vcanSam3x8e);
VLCB::NodeVariableService nvService;
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;
VLCB::Controller controller( &modconfig,
                            { &mnService, &serialUserInterface, &canService, &nvService, &ecService, &epService, &etService });  // Controller object

// module objects
VLCB::Switch moduleSwitch(16);  // an example switch as input
VLCB::LED moduleLED(17);        // an example LED as output

////////////////////////////////////////////////////////////////////////////////////////
// New policy to bring ALL headers above anything else at all.
// Maybe that is why they are called headers.
// The only exception would be defines affecting choices in a header.
////////////////////////////////////////////////////////////////////////////////////////
#define VERSION 0.1
//#define CBUS_LONG_MESSAGE
//#define CBUS_LONG_MESSAGE_MULTIPLE_LISTEN
//#define USE_EXTERNAL_EEPROM
#define DEBUG 1         // set to 0 for no serial debug
#define OLED_DISPLAY 0  // set to 0 if 128x32 OLED display is not present
#define LCD_DISPLAY 0   // set to 0 if 4x20 char LCD display is not present

#if LCD_DISPLAY
// The hd44780 library figures out what to do.  This corresponds to a display with an I2C expander pack.
// I could provide alternatives for other hardware.
hd44780_I2Cexp display(0x27);

volatile unsigned long previousTurnon = 0;
volatile unsigned long alight = 10000;
volatile boolean barGraphicsLoaded = false;
volatile boolean showingSpeeds = false;
#endif

#if DEBUG
#define DEBUG_PRINT(S) Serial << S << endl
#else
#define DEBUG_PRINT(S)
#endif


//////////////////////////////////////////////////////////////////////////

// module name, must be 7 characters, space padded.
const unsigned char PROGMEM mname[7] = { 'D', 'U', 'E', ' ', ' ', ' ', ' ' };

// forward function declarations
void eventhandler(byte, VLCB::VlcbMessage *, bool ison, byte evval);
void processSerialInput();
void printConfig();
void processModuleSwitchChange();


#ifdef CBUS_LONG_MESSAGE
///////////////////////////////////////////////////////////////////////////////////////////////
// Long message setting up.
///////////////////////////////////////////////////////////////////////////////////////////////
const byte stream_id = 13;  // This needs to be different from the ones being listened to.
// a list of stream IDs to subscribe to (this ID is defined by the sender):
byte stream_ids[] = { 11, 12, 14 };  // These are the ones which this module will read.
#ifdef CBUS_LONG_MESSAGE_MULTIPLE_LISTEN
//bool receiving[] = {false,false,false};
#endif
  // a buffer for the message fragments to be assembled into
// either sized to the maximum message length, or as much as you can afford
const unsigned int buffer_size = 128;
byte long_message_data[buffer_size];
// create a handler function to receive completed long messages:
void longmessagehandler(void *fragment, const unsigned int fragment_len, const byte stream_id, const byte status);
const byte delay_in_ms_between_messages = 50;
#endif

//
/// setup VLCB - runs once at power on from setup()
//  This now returns true if successful.
void setupVLCB()
{
  // set config layout parameters
  modconfig.EE_NVS_START = 10;
  modconfig.EE_NUM_NVS = 10;
  modconfig.EE_EVENTS_START = 50;
  modconfig.EE_MAX_EVENTS = 64;
  modconfig.EE_NUM_EVS = 1;
  //modconfig.EE_BYTES_PER_EVENT = (config.EE_NUM_EVS + 4);
  // initialise and load configuration
  controller.begin();

  const char *modeString;
  switch (modconfig.currentMode) {
    case MODE_NORMAL: modeString = "Normal"; break;
    case MODE_SETUP: modeString = "Setup"; break;
    case MODE_UNINITIALISED: modeString = "Uninitialised"; break;
    default: modeString = "Unknown"; break;
  }
  Serial << F("> mode = (") << _HEX(modconfig.currentMode) << ") " << modeString;
  Serial << F(", NN = ") << modconfig.nodeNum << endl;
  Serial << F("> mode = (") << _HEX(modconfig.currentMode) << ") " << modeString;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  VLCB::Parameters params(modconfig);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  // assign to controller
  controller.setParams(params.getParams());
  controller.setName((byte *)mname);

  // opportunity to set default NVs after module reset
  if (modconfig.isResetFlagSet()) {
    Serial << F("> module has been reset") << endl;
    modconfig.clearResetFlag();
  }

  // register our CBUS event handler, to receive event messages of learned events
  ecService.setEventHandler(eventhandler);

  // set Controller LEDs to indicate mode
  controller.indicateMode(modconfig.currentMode);

#ifdef CBUS_LONG_MESSAGE
  // subscribe to long messages and register handler
#ifdef CBUS_LONG_MESSAGE_MULTIPLE_LISTEN
  cbus_long_message.allocateContexts();
  cbus_long_message.subscribe(stream_ids, (sizeof(stream_ids) / sizeof(byte)), longmessagehandler);
#else
  cbus_long_message.subscribe(stream_ids, (sizeof(stream_ids) / sizeof(byte)), long_message_data, buffer_size, longmessagehandler);
#endif
  // this method throttles the transmission so that it doesn't overwhelm the bus:
  cbus_long_message.setDelay(delay_in_ms_between_messages);
  cbus_long_message.setTimeout(1000);
#endif

  vcanSam3x8e.setControllerInstance(0);  // only actually required for instance 1, instance 0 is the default
  if (!vcanSam3x8e.begin()) {
    DEBUG_PRINT("> error starting VLCB");
  } else {
    DEBUG_PRINT("> VLCB started");
  }
}


void setupModule()
{
}

//
/// setup - runs once at power on
//

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial << endl
         << endl
         << F("> CANDUE ** ") << __FILE__ << endl;

  setupVLCB();
  setupModule();

#if OLED_DISPLAY || LCD_DISPLAY
  initialiseDisplay();
  delay(2000);
#if OLED_DISPLAY
  displayImage(mergLogo);
  displayImage(bnhmrsLogo);
#endif

#if LCD_DISPLAY
  displayMergLogo();
  delay(2000);
#endif
#endif

  // end of setup
  DEBUG_PRINT(F("> ready"));
}

//
/// loop - runs forever
//

void loop()
{

  //
  /// do CBUS message, switch and LED processing
  //

  controller.process();

#ifdef CBUS_LONG_MESSAGE
  cbus_long_message.process();
#endif


  // Run IO_Abstraction tasks.
  // This replaces actions taken here in the previous version.
  //taskManager.runLoop();

  //
  /// bottom of loop()
  //
}

#ifdef CBUS_LONG_MESSAGE
// Example code not yet being used.
void send_a_long_message()
{
  char msg[32];
  int string_length;  // Returned by snprintf. This may exceed the actual length.
  unsigned int message_length;
  // Somewhere to send the long message.
  while (cbus_long_message.is_sending()) {}  //wait for previous message to finish.
                                             // bool cbus_long_message.sendLongMessage(char *msg, const unsigned int msg_len,
                                             //                        const byte stream_id, const byte priority = DEFAULT_PRIORITY);
  strcpy(msg, "Hello world!");
  message_length = strlen(msg);
  if (message_length > 0) {
    if (cbus_long_message.sendLongMessage((const byte *)msg, message_length, stream_id)) {
      Serial << F("long message ") << msg << F(" sent to ") << stream_id << endl;
    } else {
      Serial << F("long message sending ") << msg << F(" to ") << stream_id << F(" failed with message length ") << message_length << endl;
    }
  } else {
    Serial << F("long message preparation failed with message length ") << message_length << endl;
  }
}
#endif

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(byte index, VLCB::VlcbMessage *msg, bool ison, byte evval)
{


  return;
}

#ifdef CBUS_LONG_MESSAGE
byte new_message = true;
//
// Handler to receive a long message
//
void longmessagehandler(void *fragment, const unsigned int fragment_len, const byte stream_id, const byte status)
{
  // I need an example for what goes in here.
  //fragment[fragment_len] = 0;
  // If the message is complete it will be in fragment and I can do something with it.
  if (new_message) {  // Print this only for the start of a message.
    Serial << F("> user long message handler: stream = ") << stream_id << F(", fragment length = ")
           << fragment_len << F(", fragment = |");
    new_message = false;
  }
  if (status == CBUS_LONG_MESSAGE_INCOMPLETE) {
    // handle incomplete message
    Serial.write((char *)fragment, fragment_len);
  } else if (status == CBUS_LONG_MESSAGE_COMPLETE) {
    // handle complete message
    Serial.write((char *)fragment, fragment_len);
    Serial << F("|, status = ") << status << endl;
    new_message = true;  // reset for the next message
  } else {               // CBUS_LONG_MESSAGE_SEQUENCE_ERROR
                         // CBUS_LONG_MESSAGE_TIMEOUT_ERROR,
                         // CBUS_LONG_MESSAGE_CRC_ERROR
                         // raise an error?
    Serial << F("| Message error with  status = ") << status << endl;
    new_message = true;  // reset for the next message
  }
}

#endif

//
/// print code version config details and copyright notice
//

void printConfig(void)
{

  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Duncan Greenwood (MERG M5767) 2019") << endl;
  Serial << F("> © Martin Da Costa (MERG M6223) 2021") << endl;
  Serial << F("> © John Fletcher (MERG M6777) 2021") << endl;
  Serial << F("> © Sven Rosvall (MERG M3777) 2021") << endl;
#ifdef CBUS_LONG_MESSAGE
  Serial << F("> CBUS Long message handling available") << endl;
#ifdef CBUS_LONG_MESSAGE_MULTIPLE_LISTEN
  Serial << F("> with multiple message modification") << endl;
#endif
#endif
#ifdef USE_EXTERNAL_EEPROM
  //Serial << F("> using external EEPROM size ") << config.getEEPROMsize() << endl;
  Serial << F("> using external EEPROM") << endl;
#endif
#if OLED_DISPLAY || LCD_DISPLAY
#if OLED_DISPLAY
  Serial << F("> OLED display available") << endl;
#else
  Serial << F("> LCD display available") << endl;
#endif
#endif
  return;
}

#if (OLED_DISPLAY || LCD_DISPLAY)

void displayImage(const uint8_t *imageBitmap)
{
#if OLED_DISPLAY
  // Clear the buffer.
  display.clearDisplay();
  // Show Merg logo on the display hardware.
  display.drawBitmap(0, 0, imageBitmap, 128, 32, 1);
  display.display();
  // leave logo on screen for a while
  delay(1500);
#endif
}

void displayVersion()
{
#if OLED_DISPLAY
  // Clear the buffer.
  display.clearDisplay();
  // display module name and version for a short time
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 8);
  display.println("CANCMDDC");
  display.setTextSize(1);
  display.setCursor(60, 24);
  display.println("v2.3");
  display.display();
#endif
#if LCD_DISPLAY
  // Clear the buffer.
  display.clear();

  display.clear();
  display.setCursor(3, 0);
  display.write("CANCMDDC v");
  display.print(VERSION);
  display.setCursor(0, 1);
  display.write(char(7));
  display.setCursor(2, 1);
  display.write("David W Radcliffe &");
  display.setCursor(0, 2);
  display.write("John Fletcher with");
  display.setCursor(0, 3);
  display.write("I.Morgan & M.Riddoch");

#endif
  delay(2000);
}

void initialiseDisplay()
{
#if OLED_DISPLAY
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // Clear the buffer.
  display.clearDisplay();
#endif
#if LCD_DISPLAY
  display.begin(20, 4);  // initialize the lcd for 20 chars 4 lines, turn on backlight
  display.display();     // turn it on
  display.clear();
#endif
}

#if LCD_DISPLAY
void displayMergLogo()
{
  // Creat a set of new characters
  const uint8_t mergLogo[][8] = {
    { B00001111, B00011111, B00011111, B00011111, B00011100, B00011100, B00011100, B00011100 },  // 0
    { B00011111, B00011111, B00011111, B00011111, B00000000, B00000000, B00000000, B00000000 },  // 1
    { B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00011100, B00011100 },  // 2
    { B00000000, B00000000, B00000000, B00000000, B00011111, B00011111, B00011111, B00011111 },  // 3
    { B00000111, B00000111, B00000111, B00000111, B00000111, B00000111, B00000111, B00000111 },  // 4
    { B00000000, B00000000, B00000000, B00000000, B00011111, B00011111, B00011111, B00011111 },  // 5
    { B00011111, B00011111, B00011111, B00011111, B00001111, B00000111, B00000111, B00000111 },  // 6
    { B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111, B00011111 }   // 7
  };

  //void displayMergLogo()
  //{
  customChars(mergLogo);

  char chars[4][20] = {
    char(0), char(1), char(6), char(1), char(1), char(2), ' ', char(0), char(1), char(1), ' ', char(0), char(1), char(1), char(2), ' ', char(0), char(1), char(1), char(2),
    char(2), ' ', char(4), ' ', ' ', char(2), ' ', char(2), ' ', ' ', ' ', char(2), ' ', ' ', char(2), ' ', char(2), ' ', ' ', ' ',
    char(7), ' ', char(4), ' ', ' ', char(2), ' ', char(7), char(1), ' ', ' ', char(7), char(1), char(1), char(6), ' ', char(7), ' ', char(1), char(2),
    char(7), ' ', char(4), ' ', ' ', char(2), ' ', char(7), char(3), char(3), ' ', char(7), ' ', ' ', char(4), ' ', char(7), char(3), char(3), char(2)
  };
  displayLogo(chars);

  delay(2000);
}

void customChars(const uint8_t chars[][8])
{
  for (int i = 0; i < 8; i++) {
    display.createChar(i, (uint8_t *)chars[i]);
  }
}

#endif

void displayLogo(const char chars[4][20])
{
#if LCD_DISPLAY
  for (int i = 0; i < 4; i++) {
    display.setCursor(0, i);
    displayChars(chars[i], 20);
  }
#endif
}

void displayChars(const char chars[20], int count)
{
#if LCD_DISPLAY
  for (int j = 0; j < count; j++) {
    display.write(chars[j]);
  }
#endif
}

#endif
