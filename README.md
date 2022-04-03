# Asset_tracker_nrf - Arduino/mbed
## Functionality
This is Arduino demo firmware for custom hardware consisting of NRF52840, NRF9160 and LIS2DE12 sensor.
When movement is detected or the idle timer expires, it tries to get a GPS fix, and to
send the coordinates via UDP to a test server, that echo's them back.
For testing it is possible to fake a GPS fix.

Blue led: On: trying to get GPS fix

## Inner workings
The program uses MBED (RTOS) functions. MBED is part of Arduino nRF52 framework.
It creates several threads that run in paralel. Inter thread communication is done via a semaphore.
This semaphore is used to indicate that either movement is detected or the idle timer had expired. With this implementation with a semaphore the CPU can be 97% of the time in sleep state (not valid in debugging mode). This is NOT possible when constantly polling a boolean in the main loop to see whether movement has occurred.

The software uses among others the Sodaq LIS3DE library. This library is compatible with the LIS2DE accelerometer that is used on this board.

The program is guarded with a hardware watchdog.

<!--
## Building
### Arduino IDE
* install following libraries
    * sodaqmoja/Sodaq_LIS3DE (tested version 1.1.0)
* Install board files
    * Go to File, Preferences and set the following URL for the additional board files: http://downloads.sodaq.net/package_sodaq_index.json


### Visual Code
For building with VScode you need following (the version numbers indicate with what version development is done) 
* Visual code (1.58.2) 
    * PlatformIO plugin (version 2.3.2) for Visual code
    * Platform: Nordic nRF52 version 8.0.0
* Libraries
    * sodaqmoja/Sodaq_LIS3DE (tested version 1.1.0)
* Board files
    * put directory SODAQ_NRF in C:\Users\<user>\.platformio\packages\framework-arduino-mbed@2.0.0\variants\ (path for Windows)
-->

## Configuring
The main file contains several options to customize the behaviour. From timers to faking a GPS fix.
For debugging use is made of the library SerialDebug. See source code for instuctions how to use it.

