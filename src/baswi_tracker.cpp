#include <Arduino.h>
#define DEBUG_STREAM SerialUSB
#define MODEM_STREAM Serial2 //see pins_arduino.h
#define GPS_POLL_INTERVAL 1000 //milliseconds 
#define GPS_TIMEOUT 60000 //milliseonds

unsigned long baud = 115200;  //start at 115200

char modem_reaction[64]; //holds last modem reaction string
char longitude[32]; //holds longitude of last GPS fix
char latitude[32]; //holds latitude of last GPS fix
bool idleTimerExpired = false;

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
#include <rtos.h>

//#define Serial SerialUSB
#include <Wire.h>

#include "SparkFun_LIS2DH12.h" //Based on: http://librarymanager/All#SparkFun_LIS2DH12; adapted by baswi
SPARKFUN_LIS2DH12 accel;       //Create instance
//#include "lis2dh12.h"
//#include "lis2dh12_registers.h"

mbed::InterruptIn event1(p11); //p0.11 = ACCEL_INT1 //p11: accel.available -> INTR
mbed::InterruptIn event2(p15); //p0.15 = ACCEL_INT2 //p15: gives interrupt in movement is detected @@@sensibility to be adapted

rtos::Thread CheckModemResponse; //thread to check periodically the modem response

#define BUFF_SIZE 512
mbed::CircularBuffer<char, BUFF_SIZE> buff;
// Semaphores manage access to a shared resource
// This semaphore allows one thread to access the shared buffer
rtos::Semaphore sem(1);


volatile bool LIS_intr1_recvd = false; //LIS2DE accelerator interrupt 1; set to true in ISR
volatile bool LIS_intr2_recvd = false; //LIS2DE accelerator interrupt 2; set to true in ISR
void handle_LIS_intr1() {
  LIS_intr1_recvd = true;
}
void handle_LIS_intr2() {
  LIS_intr2_recvd = true;
}

int parse_delimited_str(char *string, char **fields, int max_fields)
{
   int i = 0;
   char delimiter = ' ';

   fields[i++] = string; //set field[0] to start of whole string

   while ((i < max_fields) && (NULL != (string = strchr(string, delimiter)))) {
      *string = '\0'; //replace delimiter by null char; finishing first field
      fields[i++] = ++string; //set next field to position after delimiter found
   }
   //no delimiter found anymore
   //no need to add trailing nul char, becuase input string already contained it
   fields[i++] = ++string;

   return --i; //number of fields found
}

// to be used if you want to print from ISR context
void PrintToBuffer(char* val)
{
  // Wait for the shared resource to be free
  sem.acquire();

  // Check, whether the buffer is empty
  // No bounds check is necessary in this case
  // overflowing data will simply overwrite the
  // same buffer
  //if(!buff.empty()) //original was tested on !buff.empty; this means that when you do 2 conseq. writes in a thread 
  //only the first one will make it
  if ((BUFF_SIZE - (buff.size())) > (strlen(val))) //max-buf-size - what-in-use > needed
  {
    // Reset the buffer
    //buff.reset();
 
    // Store the new message
    int index = 0;
    while(val[index] != '\0')
    {
      buff.push(val[index++]);
    }
    // Tell the semaphore that this thread is done writing
    sem.release();
  } else {
    // Don't forget to release the semaphore
    // Otherwise the other threads will wait forever
    sem.release();
    }
}



void GetModemReaction()
{
  int8_t index = 0;

  printV("Enter function\n\r");
  while (MODEM_STREAM.available()) 
  {
    printlnV("modem_stream.available = yes");
    modem_reaction[index++] = MODEM_STREAM.read();
    printA(modem_reaction[(index-1)]); //print reaction to console
    //PrintToBuffer(&c); //print char to circular buffer
  }

  //char_array[index] = ""; //add CRLF and null char %%%
  /*if (index > 0) {
    strcpy(&char_array[index], "\r\n\0");
    PrintToBuffer(char_array);
  }
  PrintToBuffer("Exit function\r\n");*/
}

void printDataWaitingInBuf(){
  //This function is used together with PrintToBuffer, and is necessary if you want to print from ISR context
  char data = 0;
  
  if(!buff.empty())
  {
    sem.acquire(); 
    while(buff.pop(data))
    {
      Serial.print(data);
    }
    Serial.println("");
    sem.release();
  }
}


void WriteStringToModem (char *Pmodemstring, char * Pcommentstring)
{
  //AT commands starting with # are specific part of Sodaq modem application
  //AT commands starting with + or % are standard Nrf9160 modem commands
  printlnD(Pcommentstring);
  printD(Pmodemstring); //without ln, because modem string already contains CR LF
  MODEM_STREAM.write(Pmodemstring); //turn on NB-IOT, LTE-M and GPS
  
  rtos::ThisThread::sleep_for(2000); //do not write commands too fast to modem
}

void TurnOnSodaqNRFmodem()
{
  char modemstring[64] ="";
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

  strcpy(modemstring, "AT+CCID\r\n"); //CIMI gives error; read SIM card serial number (ICCID) - which the network associates with the IMSI
  strcpy(commentstring, "Read ICCID");
  WriteStringToModem(modemstring, commentstring);

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
  char modemstring[64] ="";
  char commentstring[64] = "";

  printlnV("Enter function");

  strcpy(modemstring, "AT%XMAGPIO=1,0,0,1,1,1565,1586\r\n");
  strcpy(commentstring, "Turn on amplifier for GPS?");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xgps=1,31\r\n");
  strcpy(commentstring, "Connect to GPS");
  WriteStringToModem(modemstring, commentstring);

  //wait til gps fix; something like #XGPSP: "long 5.174879 lat 52.226059" will be received
  char *strptr = NULL;
  
  int8_t tries = 0; //number of iterations in while loop to get GPS fix
  while ((strptr == NULL) && ((tries * GPS_POLL_INTERVAL) < GPS_TIMEOUT)) //while not fix && shorter than GPS timeout
  {
    tries++;
    strptr = strstr(modem_reaction, "GPSP");
    printlnD("waiting for GPS fix");
    thread_sleep_for(GPS_POLL_INTERVAL);
    strcpy(modem_reaction, "#XGPSP: \"long 5.174879 lat 52.226059\""); //@@@only for module testing! comment in production
  }
  if (strptr == NULL) {
    printlnD("NO GPS fix found");
  } else {
    //strptr points to the start of the substring searched for
    printlnD("GPS fix found"); 
    ret = true;

    char *fields[5]; //array of strings; room for 5 fields
    parse_delimited_str(modem_reaction, fields, 5); //puts found fields in fields var
    strcpy(longitude, fields[2]); //longitude
    strcpy(latitude, fields[4]); //latitude
    latitude[strlen(latitude)-1] = '\0'; //remove last character, is ", from field; single quotes because it is a char (not string)

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
  Wire.begin();
  delay(500);

//use mebed style for attaching interrupts
  event1.rise(&handle_LIS_intr1); 
  event2.rise(&handle_LIS_intr2);

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

bool SendGPScoords()
{  
  char modemstring[128] ="";
  char commentstring[64] = "";

  printlnV("Enter function");

  strcpy(modemstring, "AT#XSOCKET=1,2,0\r\n");
  strcpy(commentstring, "Open socket <handle><protocol><role>");
  WriteStringToModem(modemstring, commentstring);
 
  //send longitude and latitude
  //strcpy(modemstring, "AT#XSENDTO=\"149.210.176.132\",12005,1,\"Test UDP 1,2,3\"\r\n"); 
  strcpy(modemstring, "AT#XSENDTO=\"149.210.176.132\",12005,1,"); 
  strcat(modemstring, "\""); //append \"
  strcat(modemstring, longitude); //append longitude
  strcat(modemstring, ", "); 
  strcat(modemstring, latitude); //append latitude
  strcat(modemstring, "\"\r\n"); //append \"
  strcpy(commentstring, "Send <url>,<port>,<datatype:1=plain text>,<data>");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "at#xsocketopt=1,20,3\r\n");
  strcpy(commentstring, "Set receive socket timeout to 3 sec");
  WriteStringToModem(modemstring, commentstring);

  strcpy(modemstring, "AT#XRECVFROM=\"149.210.176.132\",12005\r\n"); 
  strcpy(commentstring, "Receive <url>,<port>");
  WriteStringToModem(modemstring, commentstring);

  //strcpy(modemstring, "AT#XSOCKET=0\r\n"); 
  //strcpy(commentstring, "Close the socket");
  //WriteStringToModem(modemstring, commentstring);

  //@@@code to check success or failure, and do fall back 

  printlnV("Exit function");
  return true;

}


void GetGPSfixAndSendCoords()
{
  printlnV("Enter function");

  TurnOnSodaqNRFmodem();
  if (WaitForGPSfix()) {
    SendGPScoords();
  }
  TurnOffSodaqNRFmodem();
  
  printlnV("Exit function");
}


#define MODEM_POLL_TIME 100
#define STOP_FLAG 0xEF
#define MODEMPOLL_THREAD_STACK 1024 //was 224
#define IDLE_THREAD_STACK 384
static rtos::EventFlags idle_ef;
void T_getModemReaction() //mbed Thread
{
  while (1) {
    GetModemReaction();
    //Serial.println("Thread getmodemReaction");
    //thread_sleep_for(MODEM_POLL_TIME);
    rtos::ThisThread::sleep_for(1000); //put RTOS thread in to sleep
    }
}

void idle()
{
    while (1) {
        thread_sleep_for(60000);
        idleTimerExpired = true;
    }
}



void setup()
{
    rtos::Thread *modem_poll_thread = new rtos::Thread(osPriorityNormal, MODEMPOLL_THREAD_STACK, nullptr, "modem_poll_thread");
    modem_poll_thread->start(T_getModemReaction);

    rtos::Thread *idle_thread = new rtos::Thread(osPriorityNormal, IDLE_THREAD_STACK, nullptr, "idle_thread");
    idle_thread->start(idle);

  // Start communication
  DEBUG_STREAM.begin(baud);

  InitSodaqNRF();

  //baswi: printf CANNOT be used inside ISR context
  //mutexes guard calls to stdio functions, such as printf, in the Arm C standard library, and mutexes cannot be called from an ISR.
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
  //printDataWaitingInBuf(); //// check printbuffer and print contents; only neccessary if from ISR context (debug) data is written to circular buffer

  // SerialDebug handle
  // Notes: if in inactive mode (until receive anything from serial),
  // it show only messages of always or errors level type
  // And the overhead during inactive mode is very low
  // Only if not DEBUG_DISABLED

  //@Dane: how can I optimize this loop for power consumption? I have now an idle timer running in a separate mbed thread, and this loop loops contineously

  debugHandle(); 
  if (LIS_intr1_recvd == true) {
    printlnD("INTER 1 RECEIVED\n");
    LIS_intr1_recvd = false;
  }
  if (LIS_intr2_recvd || idleTimerExpired) { //Movement detected OR idle timer expired
    if (LIS_intr2_recvd) {
      printlnD("LIS2DE INTERRUPT 2 - MOVEMENT DETECTED\n");
      LIS_intr2_recvd = false; //reset interrupt received flag
    } else {
      printlnD("IDLE TIMER EXPIRED\n");
      idleTimerExpired = false;
    }
    GetGPSfixAndSendCoords(); //returns true if GPS fix coords could be sent
  }

//  delay(60000); //wait 60 seconds; every? modem action interrupts GPS@@@

  // check if the USB virtual serial wants a new baud rate
  // This will be used by the UEUpdater to flash new software
  //if (DEBUG_STREAM.baud() != baud) {
  //  baud = DEBUG_STREAM.baud();
  //  MODEM_STREAM.begin(baud);
  //}
}