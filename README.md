# sodaq_nrf
## FUNCTIONALITY
This is demo firmware for Sodaq NRF.
When movement is detected or the idle timer expires, it tries to get a GPS fix, and to
send the coordinates via UDP to a test server, that echo's them back.
For testing it is possible to fake a GPS fix.

LED's
Blue: On: trying to get GPS fix

## INNER WORKING
The program uses MBED (RTOS) functions. MBED is part of Arduino for the nRF52 framework.
It creates several threads that run in paralell. Inter thread communication is via semaphores.
A semaphore is used to indicate that either movement is detected or the idle timer had expired. With semaphore the CPU can be 97% of the time in sleep state (not in debugging mode). This is NOT possible when constantly polling a boolean in the main loop.

It uses among others the Sodaq LIS3DE library. This library is compatible with the LIS2DE accelerometer that is used on the Sodaq NRF board.

## Installing
This program is developped using Visual code and the PlatformIO plugin. If you like to compile it with the Arduino IDE you must do following:
* rename main program from .cpp to ino

## Configuring
The main file contains several options to customize the behaviour. From timers to faking a GPS fix.
For debugging use is made of the library SerialDebug. See source code for instuctions how to use it.
