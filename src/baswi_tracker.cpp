#include <Arduino.h>
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial2 //see pins_arduino.h
unsigned long baud = 115200;  //start at 115200

//--
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

// Define the initial debug level here (uncomment to do it)
#define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_VERBOSE

// Disable auto function name (good if your debug yet contains it)
//#define DEBUG_AUTO_FUNC_DISABLED true

// Include SerialDebug
#include "SerialDebug.h" // Download SerialDebug library: https://github.com/JoaoLopesF/SerialDebug
#include "mbed.h"

//#define Serial SerialUSB
#include <Wire.h>

#include "SparkFun_LIS2DH12.h" //Based on: http://librarymanager/All#SparkFun_LIS2DH12; adapted by baswi
SPARKFUN_LIS2DH12 accel;       //Create instance
//#include "lis2dh12.h"
//#include "lis2dh12_registers.h"

mbed::InterruptIn event1(p11); //p0.11 = ACCEL_INT1 //p11: accel.available -> INTR
mbed::InterruptIn event2(p15); //p0.15 = ACCEL_INT2 //p15: geeft soms bij hele wilde bewegingen INTR


volatile bool intr1_recvd = false;
volatile bool intr2_recvd = false;
void intr1_test() {
  intr1_recvd = true;
}
void intr2_test() {
  intr2_recvd = true;
}

//--
void GetModemReaction()
{
  char c;
  printlnV("Enter function");
  delay(1000); //realy necessary, otherwise data is not yet received
  while (MODEM_STREAM.available()) //write reaction from modem to debug stream
  {
    //printlnD("modem_stream.available = yes");
    //DEBUG_STREAM.write(MODEM_STREAM.read());
    c = MODEM_STREAM.read();
    printD(c);
  }
  printlnD(" ")//empty line
  printlnV("Exit function");
}

void WriteStringToModem (char *Pmodemstring, char * Pcommentstring)
{
  printlnD(Pcommentstring);
  printD(Pmodemstring); //without ln, because modem string already contains CR LF
  MODEM_STREAM.write(Pmodemstring); //turn on NB-IOT, LTE-M and GPS
  GetModemReaction();
}

void TurnOnSodaqNRFmodem()
{
  char modemstring[64] ="";
  char commentstring[64] = "";

  printlnV("Enter function");
  pinMode(NRF_ENABLE, OUTPUT);
  digitalWrite(NRF_ENABLE, HIGH);
  MODEM_STREAM.begin(baud);
  
  //AT%XSYSTEMMODE=0,0,1,0 to only turn on GPS 
  strcpy(modemstring, "AT%XSYSTEMMODE=1,1,1,0\r\n");
  strcpy(commentstring, "turn on NB-IOT, LTE-M and GPS");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CEREG=5\r\n");
  strcpy(commentstring, "get unsolicited messages");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CGDCONT: 0,\"IP\",\"data.mono\"\r\n");
  strcpy(commentstring, "set APN to Monogoto");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CFUN=1\r\n");
  strcpy(commentstring, "Set the device to full functionality");
  WriteStringToModem(modemstring, commentstring);
  
  strcpy(modemstring, "AT+CGDCONT: 0,\"IP\",\"data.mono\"\r\n");
  strcpy(commentstring, "set APN to Monogoto");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT+CGDCONT?\r\n");
  strcpy(commentstring, "Check status");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT#XSOCKET=1,2,0\r\n");
  strcpy(commentstring, "Open socket");
  WriteStringToModem(modemstring, commentstring);
 
  strcpy(modemstring, "AT#XSENDTO=\"149.210.176.132\",12005,1,\"Test UDP 1,2,3\"\r\n");
  strcpy(commentstring, "Send test data");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xsocketopt=1,20,3\r\n");
  strcpy(commentstring, "Set receive socket timeout; 20 iso 3; bug");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT#XRECVFROM=0,\"149.210.176.132\",12005\r\n");
  strcpy(commentstring, "Receive test data");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT%XMAGPIO=1,0,0,1,1,1565,1586\r\n");
  strcpy(commentstring, "Turn on amplifier for GPS?");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xgps=1,31\r\n");
  strcpy(commentstring, "Connect to GPS");
  WriteStringToModem(modemstring, commentstring);


  printlnV("Exit function");
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
  Wire.begin();
  delay(500);

//use mebed style for attaching interrupts
  event1.rise(&intr1_test); 
  event2.rise(&intr2_test);

  if (accel.begin() == false)
  {
    Serial.println("Accelerometer not detected. Check address jumper and wiring. Freezing...");
    while (100)
      ;
  }
  printlnD("Accelerometer detected"); 
  printlnV("Exit function");
}

void InitSodaqNRF()
{
  printlnV("Enter function");
  InitSodaqNRFaccel();
  printlnV("Exit function");
}

//returns true if GPS fix, false on tiemout
bool GetGPSfix()
{
  printlnV("Enter function");
  TurnOnSodaqNRFmodem();
//@@@  TurnOffSodaqNRFmodem();
  printlnV("Exit function");
}

bool SendGPScoords()
{
  printlnV("Enter function");
  printlnV("Exit function");
}


void setup()
{
  // Start communication
  DEBUG_STREAM.begin(baud);

  InitSodaqNRF();

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

  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB
  }
  printlnA("***START NRF TRACKER***");
}


void loop()
{
  // SerialDebug handle
  // Notes: if in inactive mode (until receive anything from serial),
  // it show only messages of always or errors level type
  // And the overhead during inactive mode is very low
  // Only if not DEBUG_DISABLED
  debugHandle();
  if (intr1_recvd == true) {
    printlnD("INTER 1 RECEIVED\n");
    intr1_recvd = false;
  }
  if (intr2_recvd == true) {
    printlnD("INTER 2 RECEIVED\n");
    intr2_recvd = false; //reset interrupt received flag

    GetGPSfix(); //returns true if GPS fix, false on tiemout
    SendGPScoords();
  }

  delay(60000); //wait 60 seconds; every? modem action interrupts GPS
  GetModemReaction();

  // check if the USB virtual serial wants a new baud rate
  // This will be used by the UEUpdater to flash new software
  if (DEBUG_STREAM.baud() != baud) {
    baud = DEBUG_STREAM.baud();
    MODEM_STREAM.begin(baud);
  }
  //delay(1000);
}
