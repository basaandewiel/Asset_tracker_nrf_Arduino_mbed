//TODO@@@
//    acceptance testing: measuring power consumption
//    copyright - what is needed?

// TO BE TESTED
// * NRF_DEBUG is not defined; check that GPS fix is recongized ()
//
/*
Copyright (c) 2021, SODAQ
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include <Arduino.h>
#include <Wire.h>

#include <rtos.h>
#include <Sodaq_LIS3DE.h>

#include "mbed.h"
#include "watchdog.h"
// #include "SerialDebug.h" //SerialDebug library

//****************************************
// BEGIN CUSTOMISATION
//****************************************
//To TURN OF DEBUGGING uncomment next line
#define NRF_DEBUG
// when debugging is turned on, which print statements are executed is determined by the SerialDebug library (see below)
// SerialDebug Library
//
//Simulate GPS-fix; a GPS fix is faked after turning on GPS; handy during indoor testing
//enable next line to simulate a GPS fix
#define SIMULATE_GPS
constexpr auto GPS_POLL_INTERVAL = 1000;                 //milliseconds; interval between checks for GPS fix
constexpr auto GPS_TIMEOUT = 60000;                      //milliseonds; how long wait for GPS fix before failing
constexpr auto MIN_TIME_BETWEEN_ACCEL_INTERRUPTS = 5000; //milleseconds, minimum time between handling of accel interrupts

constexpr auto DESTINATION_IP = "149.210.176.132"; //destination to send GPS coordinates to
constexpr auto DESTINATION_PORT = "12005";         //port to send GPS coordinates to

constexpr auto WATCHDOGTIMEOUT = 2949120; //(90 seconds * 32768)+1 //watchdogtimer must be > than idle_timer
// this is built in watchdog in nRF52; resets chip when it expires.
constexpr auto IDLE_TIMER = 30; //seconds; time after wich it is tried to get GPS fix and  send coordinates to destination
//****************************************
// END CUSTOMISATION
//****************************************

// Disable all debug ? Good to release builds (production)
// as nothing of SerialDebug is compiled, zero overhead :-)
// For it just uncomment the DEBUG_DISABLED
//#define DEBUG_DISABLED true
#ifndef NRF_DEBUG
// Disable SerialDebug debugger ? No more commands and features as functions and globals
// Uncomment this to disable it
#define DEBUG_DISABLE_DEBUGGER true
#else //debugging on
// Define the initial debug level here (uncomment to do it)
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_DEBUG
// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true

// SerialDebug Library
// Disable all debug ? Good to release builds (production)
// For it just uncomment the DEBUG_DISABLED
//#define DEBUG_DISABLED true

// Disable SerialDebug debugger ? No more commands and features as functions and globals
// Uncomment this to disable it
//#define DEBUG_DISABLE_DEBUGGER true

// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true
#endif


#ifdef NRF_DEBUG
  #define  printDebug 1
#else
  #define  printDebug 0
#endif

#define debugPrint(x) if (printDebug) { DEBUG_STREAM.print(x); }
#define debugPrintln(x) if (printDebug) { DEBUG_STREAM.println(x); }

Sodaq_LIS3DE accelerometer(Wire, (uint8_t)0x19);

#define LED_BLUE_PIN (24u)
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial2

Watchdog watchdog; // defining an object of type watchdog

//#define Serial SerialUSB

constexpr unsigned long baud = 115200; //start at 115200

std::string modem_reaction;

char longitude[15];            //longitude of last GPS fix
char latitude[15];             //latitude of last GPS fix
us_timestamp_t timeLastGPSfix; //timestap of last GPS fix

// Threshold for interrupt trigger
double threshold = -0.8;

rtos::Semaphore sem_getSendGPScoords(0, 1); //0: default= not needed to get and send GPS coords
//1= necessary to get GPS fix; if interrupt and idle timer release the semaphore, it is still 1, and not 2.

void interrupt_event() //LIS2DE interrupt; movement detected
{
  sem_getSendGPScoords.release(); //enable getting and sending GPS coords
}

void parse_delimited_stdstr(std::string inputstring, char *fields[5], int max_fields)
{
  //split inputstring if fields separated by <space> and put result in fields
  debugPrintln("ENTER");

  char *cstr = new char[inputstring.length() + 1];
  std::strcpy(cstr, inputstring.c_str());

  uint8_t fieldnbr = 0;
  char *p = std::strtok(cstr, " ");
  while ((fieldnbr < max_fields) && (p != 0))
  {
    fields[fieldnbr] = p;
    p = std::strtok(NULL, " ");
    fieldnbr++;
  }
  delete[] cstr;
  debugPrintln("EXIT");
}

void GetModemReaction()
{
  //Get reaction of modem to last sent modemcommand
  //this function is called from mbed thread (only when debugging is 'on')
  //debugPrint("Enter function\n\r");
  while (MODEM_STREAM.available())
  {
    modem_reaction.push_back(MODEM_STREAM.read());
    debugPrint(modem_reaction.back());
  }
}

void WriteStringToModem(std::string Pmodemstring, std::string Pcommentstring)
{
  //write Pmodmestring to modem; Pcommentstring is only for debugging
  char tmpStr[64] = "";
  //AT commands starting with # are specific part of Sodaq modem application
  //AT commands starting with + or % are standard Nrf9160 modem commands
  debugPrintln(Pcommentstring.c_str());
  debugPrintln(Pmodemstring.c_str());
  MODEM_STREAM.print(Pmodemstring.c_str());
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(250)); //do not write commands too fast to modem
}

void TurnOnSodaqNRFmodem()
{
  std::string modemstring = "";
  modemstring.reserve(64);

  std::string commentstring = "";
  commentstring.reserve(64);

  debugPrintln("Enter function");
  pinMode(NRF_ENABLE, OUTPUT); //enable modem
  digitalWrite(NRF_ENABLE, HIGH);
  MODEM_STREAM.begin(baud);

  //AT%XSYSTEMMODE=<LTE_M_support>,<NB_IoT_support>,<GNSS_support>,<LTE_preference>
  //AT%XSYSTEMMODE=0,0,1,0 to only turn on GPS
  //%XSYSTEMMODE=1,1,0,2 trun on LTE-M and NB-IOT, preference is NB-IOT

  modemstring = "AT%XSYSTEMMODE=1,1,1,2\r\n";
  commentstring = "PRE: radio off; turn on LTE-M, NB-IOT, GPS, networkpreference";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "AT+CEREG=5\r\n";
  commentstring = "get unsolicited messages";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "AT+CFUN=1\r\n";
  commentstring = "Set the device to full functionality; NB:at+cfun=0 writes NVM";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "AT+CIMI\r\n"; //CIMI gives error; read SIM card serial number (ICCID) - which the network associates with the IMSI
  commentstring = "Read IMSI";
  WriteStringToModem(modemstring, commentstring);

  //  strcpy(modemstring, "AT+CCID\r\n"); //CIMI gives error; read SIM card serial number (ICCID) - which the network associates with the IMSI
  //  strcpy(commentstring, "Read ICCID");
  //  WriteStringToModem(modemstring, commentstring);

  modemstring = "AT+CGDCONT=0,\"IP\",\"data.mono\"\r\n";
  commentstring = "set APN to Monogoto";
  WriteStringToModem(modemstring, commentstring);
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(5000));

  modemstring = "AT+CGDCONT?\r\n";
  commentstring = "Check status; returns cid,IP,APN,IP-adr,0,0";
  WriteStringToModem(modemstring, commentstring);

  debugPrintln("Exit function");
}

bool WaitForGPSfix()
//if (gps fix found) return TRUE; and global vars longitude and latitude contain coordinates
//else return false;
{
  bool ret = false; //default returnvalue
  std::string modemstring = "";
  modemstring.reserve(64); //avoid heap fragmentation

  std::string commentstring = "";
  commentstring.reserve(64); //avoid heap fragmentation

  debugPrintln("Enter function");

  modemstring = "AT%XMAGPIO=1,0,0,1,1,1565,1586\r\n";
  commentstring = "Turn on amplifier for GPS";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "at#xgps=1,31\r\n";
  commentstring = "Connect to GPS";
  WriteStringToModem(modemstring, commentstring);

  //wait til gps fix; something like #XGPSP: "long 5.174879 lat 52.226059" will be received
  std::size_t foundpos = std::string::npos;

  int8_t tries = 0;                                                                      //number of iterations in while loop to get GPS fix
  while ((foundpos == std::string::npos) && ((tries * GPS_POLL_INTERVAL) < GPS_TIMEOUT)) //while no fix && shorter than GPS timeout
  {
#ifndef NRF_DEBUG
    GetModemReaction(); //If debug is on, modem reaction is retrieved contineously; if debug is off
                        //we need to call to get the modem reaction
#endif
    tries++;
    foundpos = modem_reaction.find("GPSP");
    debugPrintln("waiting for GPS fix");
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(GPS_POLL_INTERVAL));
#ifdef SIMULATE_GPS
    modem_reaction = "#XGPSP: \"long 5.174879 lat 52.226059\"";
#endif
  }
  if (foundpos == std::string::npos)
  {
    debugPrintln("NO GPS fix found");
  }
  else
  {
    debugPrintln("GPS fix found");
    ret = true;

    char *fields[5];
    parse_delimited_stdstr(modem_reaction, fields, 5); //puts found fields in fields var
    strcpy(longitude, fields[2]);                      //longitude

    strcpy(latitude, fields[4]);           //longitude
    latitude[strlen(latitude) - 1] = '\0'; //remove last character, is ", from field; single quotes because it is a char (not string)
  }
  //turn off GPS
  modemstring = "at#xgps=0\r\n";
  commentstring = "Switch off GPS; no need to switch off GPS amplifier";
  WriteStringToModem(modemstring, commentstring);

  debugPrintln("Exit function");
  return ret;
}

void TurnOffSodaqNRFmodem()
{
  debugPrintln("Enter function");
  MODEM_STREAM.write("AT+CFUN=4\r\n"); //Sets the device to flight mode
  MODEM_STREAM.end();
  pinMode(NRF_ENABLE, OUTPUT);
  digitalWrite(NRF_ENABLE, LOW);
  debugPrintln("Exit function");
}

void InitSodaqNRFaccel()
{
  //initialize LIS2DE accelerometer
  debugPrintln("Enter function");
  Wire.begin(); //Start the I2C bus

  pinMode(ACCEL_INT1, INPUT);
  attachInterrupt(ACCEL_INT1, interrupt_event, CHANGE);

  accelerometer.enable(true,
                       Sodaq_LIS3DE::NormalLowPower10Hz,
                       Sodaq_LIS3DE::XYZ,
                       Sodaq_LIS3DE::Scale8g,
                       true);

  delay(100);

  accelerometer.enableInterrupt1(
      Sodaq_LIS3DE::XHigh | Sodaq_LIS3DE::XLow | Sodaq_LIS3DE::YHigh | Sodaq_LIS3DE::YLow | Sodaq_LIS3DE::ZHigh | Sodaq_LIS3DE::ZLow,
      20 * 2.0 / 100.0, //=0.4
      0,
      Sodaq_LIS3DE::MovementRecognition);

  debugPrintln("Exit function");
}

void InitSodaqNRF()
{
  debugPrintln("Enter function");
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, HIGH); //turn blue led off

  watchdog.init(WATCHDOGTIMEOUT); //init watchdog

  InitSodaqNRFaccel();
  debugPrintln("Exit function");
}

bool SendGPScoords()
{
  //send GPS coords to configured IP and portnr
  std::string modemstring = "";
  modemstring.reserve(64);

  std::string commentstring = "";
  modemstring.reserve(64);

  debugPrintln("Enter function");

  modemstring = "AT#XSOCKET=1,2,0\r\n";
  commentstring = "Open socket <handle><protocol><role>";
  WriteStringToModem(modemstring, commentstring);

  //send longitude and latitude
  //create string "AT#XSENDTO="IP-number",<port number>,1,"<Longitude>, <Latitude>"\r\n");
  modemstring = "AT#XSENDTO=\"";
  modemstring.append(DESTINATION_IP);
  modemstring.append("\",");
  modemstring.append(DESTINATION_PORT);
  modemstring.append(",1,");
  modemstring.append("\"");
  modemstring.append(longitude);
  modemstring.append(", ");
  modemstring.append(latitude);
  modemstring.append("\"\r\n");
  commentstring = "Send <url>,<port>,<datatype:1=plain text>,<data>";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "at#xsocketopt=1,20,3\r\n";
  commentstring = "Set receive socket timeout to 3 sec";
  WriteStringToModem(modemstring, commentstring);

  //only necessary for testing when UDP echo is configured on server
  //create string "AT#XRECVFROM="IP-number",<port number>\r\n"
  modemstring = "AT#XRECVFROM=\"";
  modemstring.append(DESTINATION_IP);
  modemstring.append("\",");
  modemstring.append(DESTINATION_PORT);
  modemstring.append("\r\n");
  commentstring = "Receive <url>,<port>";
  WriteStringToModem(modemstring, commentstring);

  modemstring = "AT#XSOCKET=0\r\n";
  commentstring = "Close the socket";
  WriteStringToModem(modemstring, commentstring);

  debugPrintln("Exit function");
  return true;
}

void GetGPSfixAndSendCoords()
{
  debugPrintln("Enter function");
  digitalWrite(LED_BLUE_PIN, LOW); //turn on blue led to indicate trying to get GPS fix

  TurnOnSodaqNRFmodem();
  if (WaitForGPSfix())
  {
    SendGPScoords();
    timeLastGPSfix = mbed_uptime(); //save time of last GPS fix and coords sent
  }
  TurnOffSodaqNRFmodem();
  digitalWrite(LED_BLUE_PIN, HIGH); //turn of blue led

  debugPrintln("Exit function");
}

constexpr auto MODEM_POLL_TIME = 200;
constexpr auto MODEMPOLL_THREAD_STACK = 1024;
constexpr auto IDLE_THREAD_STACK = 1024;

void T_getModemReaction() //mbed Thread
{
  while (1)
  {
    GetModemReaction();
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(MODEM_POLL_TIME)); //put RTOS thread in to sleep
  }
}

void idle()
{
  us_timestamp_t timeSinceLastGPSfix;
  while (1)
  {
    debugPrintln("idle thread - ENTER");

    rtos::ThisThread::sleep_for(std::chrono::seconds(IDLE_TIMER)); //put RTOS thread in to sleep; so it doesn't fire directly after thread craetion
    debugPrintln("***IDLE TIMER EXPIRED***");

    timeSinceLastGPSfix = mbed_uptime() - timeLastGPSfix; //micro seconds
    debugPrint("timesincelastGPSfix: ");
    debugPrintln(timeSinceLastGPSfix);

    if (timeSinceLastGPSfix < (IDLE_TIMER * 1000000)) //IDLE_TIMER is in seconds
    {                                                 //do not send GPS coords sooner than after IDLE_TIMER msec since last time
      debugPrint("Extra sleep for idle timer (in msec): ");
      debugPrintln((IDLE_TIMER * 1000 - timeSinceLastGPSfix / 1000));
      rtos::ThisThread::sleep_for(std::chrono::milliseconds(IDLE_TIMER * 1000 - timeSinceLastGPSfix / 1000));
    }

    us_timestamp_t timeInSleep = mbed_time_sleep(); //Provides the time spent in sleep mode since boot.
    us_timestamp_t uptime = mbed_uptime();
    debugPrint("Percentage in sleep since boot: ");
    debugPrintln((uint8_t)(timeInSleep * 100 / uptime));

    sem_getSendGPScoords.release(); //enable getting and sending GPS coords
  }
}

void setup()
{
  modem_reaction.reserve(255); //avoid heap fragmentation
#ifdef NRF_DEBUG
  // debugSetLevel(DEBUG_LEVEL_DEBUG);
  Serial.begin(baud); //only open and wait for console if NRF_DEBUG is defined
  while (!DEBUG_STREAM)
    ;
  //only check modem reaction contineously when debugging
  rtos::Thread *modem_poll_thread = new rtos::Thread(osPriorityNormal, MODEMPOLL_THREAD_STACK, nullptr, "modem_poll_thread");
  modem_poll_thread->start(T_getModemReaction);
#endif

  rtos::Thread *idle_thread = new rtos::Thread(osPriorityNormal, IDLE_THREAD_STACK, nullptr, "idle_thread");
  idle_thread->start(idle);

  InitSodaqNRF();
  timeLastGPSfix = mbed_uptime();

#ifndef DEBUG_DISABLE_DEBUGGER
// Add Functions and global variables to SerialDebug
// Add functions that can called from SerialDebug

//debugAddFunctionVoid(F("function"), &function); // Example for function without args
//debugAddFunctionStr(F("function"), &function); // Example for function with one String arg
//debugAddFunctionInt(F("function"), &function); // Example for function with one int arg

// Add global variables that can showed/changed from SerialDebug
// Note: Only global, if pass local for SerialDebug, can be dangerous

//  debugAddGlobalInt(F("a"), &a);
//  debugAddGlobalInt(F("b"), &b);
//  debugAddGlobalInt(F("c"), &c);
#endif // DEBUG_DISABLE_DEBUGGER

  debugPrintln("***START NRF TRACKER***");
  debugPrintln(" ");
  debugPrintln(" ");
}

void loop()
{
#ifdef NRF_DEBUG
  // debugHandle(); //handle interactive debug settings/actions6
#endif
  sem_getSendGPScoords.acquire(); //wait till requested to get and send GPS coords
  GetGPSfixAndSendCoords();

  //wait to prevent handing movement interrupts too fast
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(MIN_TIME_BETWEEN_ACCEL_INTERRUPTS));
  sem_getSendGPScoords.try_acquire_for(std::chrono::milliseconds(10)); //discard accel interrupts that occurred in the mean time

  watchdog.reload(); //kick watchdog
}
