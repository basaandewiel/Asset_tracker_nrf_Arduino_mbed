# sodaq_nrf
This is demo firmware for Sodaq NRF.
When movement is detected or the idle timer expires, it tries to get a GPS fix, and to
send the coordinates via UDP to a test server, that echo's them back

LED's
Blue: On: trying to get GPS fix
