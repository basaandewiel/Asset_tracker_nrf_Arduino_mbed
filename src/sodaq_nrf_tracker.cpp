//TODO@@@
//    fix compiler warnings
//    acceptance testing: measuring power consumption
//    WAITING for correct board files
//
/*
Copyright (c) 2016-21, SODAQ
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
#define GPS_POLL_INTERVAL 1000 //milliseconds; interval between checks for GPS fix
#define GPS_TIMEOUT 60000      //milliseonds; how long wait for GPS fix before failing
#define MINTIMEBETWEENACCELINTERRUPTS 5000 //milleseconds, minimum time between handling of accel interrupts

#define DESTINATION_IP "149.210.176.132" //destination to send GPS coordinates to
#define DESTINATION_PORT "12005"         //port to send GPS coordinates to
#define WATCHDOGTIMEOUT 2949120          //(90 seconds * 32768)+1 //watchdogtimer must be > than idle_timer
// this is built in watchdog in nRF52; resets chip when it expires.
#define IDLE_TIMER 30000 //milliseconds; time after wich it is tried to get GPS fix and  send coordinates to destination
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
#else
// Define the initial debug level here (uncomment to do it)
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_DEBUG
// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true
// Include SerialDebug
#endif
#include "SerialDebug.h" // Download SerialDebug library: https://github.com/JoaoLopesF/SerialDebug
#include "mbed.h"
#include <rtos.h>
#include <Sodaq_LIS3DE.h>
Sodaq_LIS3DE accelerometer(Wire, (uint8_t)0x19);

//*** serial debug***
// Example from ladyada.net/learn/arduino/lesson4.html
int a = 5;
int b = 10;
int c = 20;

// SerialDebug Library
// Disable all debug ? Good to release builds (production)
// as nothing of SerialDebug is compiled, zero overhead :-)
// For it just uncomment the DEBUG_DISABLED
//#define DEBUG_DISABLED true

// Disable SerialDebug debugger ? No more commands and features as functions and globals
// Uncomment this to disable it
//#define DEBUG_DISABLE_DEBUGGER true

// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true

// Include SerialDebug
#include "SerialDebug.h" // Download SerialDebug library: https://github.com/JoaoLopesF/SerialDebug
#include "mbed.h"
#include <rtos.h>

#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial2 //210728 serial2 works with board files from JW

unsigned long baud = 115200; //start at 115200

char modem_reaction[64]; //holds last modem reaction string
char longitude[32];      //holds longitude of last GPS fix
char latitude[32];       //holds latitude of last GPS fix
us_timestamp_t timeLastGPSfix;

#include "watchdog.h"
Watchdog watchdog; // defining an object of type watchdog

//#define Serial SerialUSB
#include <Wire.h>

// Print the debug information on the SerialUSB
#define USB SerialUSB

// Threshold for interrupt trigger
double threshold = -0.8;
int now = millis();
//volatile bool interrupt_received = false;

rtos::Semaphore sem_getSendGPScoords(0, 1); //0: default not needed to get and send GPS coords
//1: maximum number of resoures; if interrupt and idle timer releae the semaphore, it is still 1, and not 2.

void interrupt_event()
{
  //Board flipped
  sem_getSendGPScoords.release(); //enable getting and sending GPS coords
}

int parse_delimited_str(char *string, char **fields, int max_fields)
{
  int i = 0;
  char delimiter = ' ';

  fields[i++] = string; //set field[0] to start of whole string

  while ((i < max_fields) && (NULL != (string = strchr(string, delimiter))))
  {
    *string = '\0';         //replace delimiter by null char; finishing first field
    fields[i++] = ++string; //set next field to position after delimiter found
  }
  //no delimiter found anymore
  //no need to add trailing nul char, becuase input string already contained it
  fields[i++] = ++string;

  return --i; //number of fields found
}

void GetModemReaction()
{
  int8_t index = 0;

  printV("Enter function\n\r");
  while (MODEM_STREAM.available())
  {
    modem_reaction[index++] = MODEM_STREAM.read();
    printD(modem_reaction[(index - 1)]); //print reaction to console
  }
}

void WriteStringToModem(char *Pmodemstring, char *Pcommentstring)
{
  //AT commands starting with # are specific part of Sodaq modem application
  //AT commands starting with + or % are standard Nrf9160 modem commands
  printlnD(Pcommentstring);
  printD(Pmodemstring);             //without ln, because modem string already contains CR LF
  MODEM_STREAM.write(Pmodemstring); //turn on NB-IOT, LTE-M and GPS

  rtos::ThisThread::sleep_for(250); //do not write commands too fast to modem
}

void TurnOnSodaqNRFmodem()
{
  char modemstring[64] = "";
  char commentstring[64] = "";

  printlnV("Enter function");
  pinMode(NRF_ENABLE, OUTPUT);
  digitalWrite(NRF_ENABLE, HIGH);
  MODEM_STREAM.begin(baud);

  //AT%XSYSTEMMODE=<LTE_M_support>,<NB_IoT_support>,<GNSS_support>,<LTE_preference>
  //AT%XSYSTEMMODE=0,0,1,0 to only turn on GPS
  //%XSYSTEMMODE=1,1,0,2 trun on LTE-M and NB-IOT, preference is NB-IOT

  strcpy(modemstring, "AT%XSYSTEMMODE=1,1,1,2\r\n");
  strcpy(commentstring, "PRE: radio off; turn on LTE-M, NB-IOT, GPS, networkpreference");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CEREG=5\r\n");
  strcpy(commentstring, "get unsolicited messages");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CFUN=1\r\n");
  strcpy(commentstring, "Set the device to full functionality; NB:at+cfun=0 writes NVM");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CIMI\r\n"); //CIMI gives error; read SIM card serial number (ICCID) - which the network associates with the IMSI
  strcpy(commentstring, "Read IMSI");
  WriteStringToModem(modemstring, commentstring);

  //  strcpy(modemstring, "AT+CCID\r\n"); //CIMI gives error; read SIM card serial number (ICCID) - which the network associates with the IMSI
  //  strcpy(commentstring, "Read ICCID");
  //  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CGDCONT=0,\"IP\",\"data.mono\"\r\n");
  strcpy(commentstring, "set APN to Monogoto");
  WriteStringToModem(modemstring, commentstring);
  rtos::ThisThread::sleep_for(5000);

  strcpy(modemstring, "AT+CGDCONT?\r\n");
  strcpy(commentstring, "Check status; returns cid,IP,APN,IP-adr,0,0");
  WriteStringToModem(modemstring, commentstring);

  printlnV("Exit function");
}

bool WaitForGPSfix()
//POST: if (gps fix found) return TRUE; and global vars longitude and latitude contain coordinates
//      else return false;
{
  bool ret = false; //default returnvalue
  char modemstring[64] = "";
  char commentstring[64] = "";

  printlnV("Enter function");

  strcpy(modemstring, "AT%XMAGPIO=1,0,0,1,1,1565,1586\r\n");
  strcpy(commentstring, "Turn on amplifier for GPS");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xgps=1,31\r\n");
  strcpy(commentstring, "Connect to GPS");
  WriteStringToModem(modemstring, commentstring);

  //wait til gps fix; something like #XGPSP: "long 5.174879 lat 52.226059" will be received
  char *strptr = NULL;

  int8_t tries = 0;                                                       //number of iterations in while loop to get GPS fix
  while ((strptr == NULL) && ((tries * GPS_POLL_INTERVAL) < GPS_TIMEOUT)) //while not fix && shorter than GPS timeout
  {
    tries++;
    strptr = strstr(modem_reaction, "GPSP");
    printlnD("waiting for GPS fix");
    thread_sleep_for(GPS_POLL_INTERVAL);
#ifdef SIMULATE_GPS
    strcpy(modem_reaction, "#XGPSP: \"long 5.174879 lat 52.226059\""); //simulate GPS fix for testing
#endif
  }
  if (strptr == NULL)
  {
    printlnD("NO GPS fix found");
  }
  else
  {
    //strptr points to the start of the substring searched for
    printlnD("GPS fix found");
    ret = true;

    char *fields[5];                                //array of strings; room for 5 fields
    parse_delimited_str(modem_reaction, fields, 5); //puts found fields in fields var
    strcpy(longitude, fields[2]);                   //longitude
    strcpy(latitude, fields[4]);                    //latitude
    latitude[strlen(latitude) - 1] = '\0';          //remove last character, is ", from field; single quotes because it is a char (not string)

    printlnD(longitude);
    printlnD(latitude);
  }
  //turn off GPS
  strcpy(modemstring, "at#xgps=0\r\n");
  strcpy(commentstring, "Switch off GPS; no need to switch off GPS amplifier");
  WriteStringToModem(modemstring, commentstring);

  printlnV("Exit function");
  return ret;
}

void TurnOffSodaqNRFmodem()
{
  printlnV("Enter function");
  MODEM_STREAM.write("AT+CFUN=4\r\n"); //Sets the device to flight mode
  MODEM_STREAM.end();
  pinMode(NRF_ENABLE, OUTPUT);
  digitalWrite(NRF_ENABLE, LOW);
  printlnV("Exit function");
}

void InitSodaqNRFaccel()
{
  printlnV("Enter function");
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

  printlnD("Accelerometer detected");
  printlnV("Exit function");
}

// These define's must be placed at the beginning before #include "NRF52TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// For Nano33-BLE, don't use Serial.print() in ISR as system will definitely hang.
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define LED_BLUE_PIN (24u)

#define TIMER0_INTERVAL_MS 30000 //30 sec

void InitSodaqNRF()
{
  printlnV("Enter function");
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_BLUE_PIN, HIGH); //turn blue led off

  watchdog.init(WATCHDOGTIMEOUT); //init watchdog

  InitSodaqNRFaccel();
  printlnV("Exit function");
}

bool SendGPScoords()
{
  char modemstring[128] = "";
  char commentstring[64] = "";

  printlnV("Enter function");

  strcpy(modemstring, "AT#XSOCKET=1,2,0\r\n");
  strcpy(commentstring, "Open socket <handle><protocol><role>");
  WriteStringToModem(modemstring, commentstring);

  //send longitude and latitude
  //create string "AT#XSENDTO="IP-number",<port number>,1,"<Longitude>, <Latitude>"\r\n");
  strcpy(modemstring, "AT#XSENDTO=\"");
  strcat(modemstring, DESTINATION_IP);
  strcat(modemstring, "\",");
  strcat(modemstring, DESTINATION_PORT);
  strcat(modemstring, ",1,");

  strcat(modemstring, "\"");      //append \"
  strcat(modemstring, longitude); //append longitude
  strcat(modemstring, ", ");
  strcat(modemstring, latitude); //append latitude
  strcat(modemstring, "\"\r\n"); //append \"
  strcpy(commentstring, "Send <url>,<port>,<datatype:1=plain text>,<data>");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xsocketopt=1,20,3\r\n"); //only necessary for testing when UDP echo is configured on server
  strcpy(commentstring, "Set receive socket timeout to 3 sec");
  WriteStringToModem(modemstring, commentstring);

  //only necessary for testing when UDP echo is configured on server
  //create string "AT#XRECVFROM="IP-number",<port number>\r\n"
  strcpy(modemstring, "AT#XRECVFROM=\""); //only necessary for testing when UDP echo is configured on server
  strcat(modemstring, DESTINATION_IP);
  strcat(modemstring, "\",");
  strcat(modemstring, DESTINATION_PORT);
  strcat(modemstring, "\r\n");
  strcpy(commentstring, "Receive <url>,<port>");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT#XSOCKET=0\r\n");
  strcpy(commentstring, "Close the socket");
  WriteStringToModem(modemstring, commentstring);

  printlnV("Exit function");
  return true;
}

void GetGPSfixAndSendCoords()
{
  printlnV("Enter function");
  digitalWrite(LED_BLUE_PIN, LOW); //turn on blue led to indicate trying to get GPS fix

  TurnOnSodaqNRFmodem();
  if (WaitForGPSfix())
  {
    SendGPScoords();
    timeLastGPSfix = mbed_uptime(); //save time of last GPS fix and coords sent
  }
  TurnOffSodaqNRFmodem();
  //thread_sleep_for(1000); //signal timeout via blue led
  digitalWrite(LED_BLUE_PIN, HIGH); //turn of led

  printlnV("Exit function");
}

#define MODEM_POLL_TIME 200
#define MODEMPOLL_THREAD_STACK 1024 //was 224
#define IDLE_THREAD_STACK 1024      //was 384

void T_getModemReaction() //mbed Thread
{
  while (1)
  {
    GetModemReaction();
    rtos::ThisThread::sleep_for(MODEM_POLL_TIME); //put RTOS thread in to sleep
  }
}

void idle()
{
  us_timestamp_t timeSinceLastGPSfix;
  while (1)
  {
    printlnV("idle thread - ENTER");
    rtos::ThisThread::sleep_for(IDLE_TIMER); //put RTOS thread in to sleep; so it doesn't fire directly after thread craetion
    printlnA("***IDLE TIMER EXPIRED***");

    timeSinceLastGPSfix = mbed_uptime() - timeLastGPSfix;
    if (timeSinceLastGPSfix < (IDLE_TIMER * 1000))
    { //do not send GPS coords sooner than after IDLE_TIMER msec since last time
      printD("timesincelastGPSfix: ");
      printlnD(timeSinceLastGPSfix);

      printD("Extra sleep for idle timer (in msec): ");
      printlnD((IDLE_TIMER - timeSinceLastGPSfix / 1000));
      rtos::ThisThread::sleep_for(IDLE_TIMER - timeSinceLastGPSfix / 1000);
    }

    us_timestamp_t timeInSleep = mbed_time_sleep();         //Provides the time spent in sleep mode since boot.
    us_timestamp_t uptime = mbed_uptime();
    printA("Percentage in sleep since boot: ");
    printlnA((uint8_t)(timeInSleep * 100 / uptime));

    sem_getSendGPScoords.release(); //enable getting and sending GPS coords
  }
}

void setup()
{
#ifdef NRF_DEBUG 
  debugSetLevel(DEBUG_LEVEL_DEBUG);
  Serial.begin(115200); //only open and wait for console if NRF_DEBUG is defined
  while (!Serial)
    ;
  //only check modem reaction when debugging
  rtos::Thread *modem_poll_thread = new rtos::Thread(osPriorityNormal, MODEMPOLL_THREAD_STACK, nullptr, "modem_poll_thread");
  modem_poll_thread->start(T_getModemReaction);
#endif

  rtos::Thread *idle_thread = new rtos::Thread(osPriorityNormal, IDLE_THREAD_STACK, nullptr, "idle_thread");
  idle_thread->start(idle);

  InitSodaqNRF();
  timeLastGPSfix = mbed_uptime();

  // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
  // This has to be done after the first call to attachInterrupt()
  /*GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;*/
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

  printlnD("***START NRF TRACKER***");
  printlnD(" ");
  printlnD(" ");
}

void loop()
{
#ifdef NRF_DEBUG
  debugHandle(); //handle interactive debug settings/actions6
#endif

  sem_getSendGPScoords.acquire(); //wait till requested to get and send GPS coords
//  detachInterrupt(ACCEL_INT1); //detaching and reattaching interrupt causes program to hang; so use another solution
  GetGPSfixAndSendCoords();       //returns true if GPS fix coords could be sent
  thread_sleep_for(MINTIMEBETWEENACCELINTERRUPTS); //wait to prevent handing movement interrupts too fast
  sem_getSendGPScoords.try_acquire_for(10); //discard accel interrupts that occurred in the mean time

  watchdog.reload();              //kick watchdog
}
