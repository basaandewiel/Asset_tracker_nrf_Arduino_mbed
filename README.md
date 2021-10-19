# sodaq_nrf - power friendly tracker
## Functionality
This is demo firmware for Sodaq NRF.
When movement is detected or the idle timer expires, it tries to get a GPS fix, and to
send the coordinates via UDP to a test server, that echo's them back.
For testing it is possible to fake a GPS fix.

LED's
Blue: On: trying to get GPS fix

## Inner workings
The program uses MBED (RTOS) functions. MBED is part of Arduino for the nRF52 framework.
It creates several threads that run in paralell. Inter thread communication is done via asemaphore.
This semaphore is used to indicate that either movement is detected or the idle timer had expired. With semaphore the CPU can be 97% of the time in sleep state (not valid in debugging mode). This is NOT possible when constantly polling a boolean in the main loop to see whether movement has occurred.

It uses among others the Sodaq LIS3DE library. This library is compatible with the LIS2DE accelerometer that is used on the Sodaq NRF board.

The program is guarded with a hardware watchdog.

## Building
### Visual Code
For building you need following (the version numbers indicate with what version development is done) 
* Visual code (1.58.2) 
    * PlatformIO plugin (version 2.3.2) for Visual code
    * Platform: Nordic nRF52 version 8.0.0
* Libraries
    * joaolopesf/SerialDebug (tested version 0.9.82)
    * sodaqmoja/Sodaq_LIS3DE (tested version 1.1.0)
* Board files
    * put directory SODAQ_NRF in C:\Users\<user>\.platformio\packages\framework-arduino-mbed@2.0.0\variants\ (path for Windows)


### Arduino IDE
If you like to build it with the Arduino IDE you must do following:
* rename .../sodaq_nrf_tracker/src/sodaq_nrf_tracker.cpp to .../sodaq_nrf_tracker/src/sodaq_nrf_tracker.ino
* move sodaq_nrf_tracker.ino to ../sodaq_nrf_tracker/sodaq_nrf_tracker.ino (directory abover scr directory)
* install following libraries
    * joaolopesf/SerialDebug (tested version 0.9.82)
    * sodaqmoja/Sodaq_LIS3DE (tested version 1.1.0)
* Install board files
    * Go to File, Preferences and set the following URL for the additional board files: http://downloads.sodaq.net/package_sodaq_index.json


## Configuring
The main file contains several options to customize the behaviour. From timers to faking a GPS fix.
For debugging use is made of the library SerialDebug. See source code for instuctions how to use it.

## License
Copyright (c) 2021, SODAQ All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
