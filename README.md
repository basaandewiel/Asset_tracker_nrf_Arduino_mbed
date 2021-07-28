# sodaq_nrf
This is demo firmware for Sodaq NRF.
When movement is detected or the idle timer expires, it tries to get a GPS fix, and to
send the coordinates via UDP to a test server, that echo's them back

LED's
Blue: On: trying to get GPS fix

The program makes use of mbed and rtos; 
Different threads run parallel, and communicate with each other via semaphores.
Movement detection generates an interrupt on the nRF52.
This results in a power efficient program (CPU is 97% of time in sleep).

The main file contains several options to customize the behaviour.
