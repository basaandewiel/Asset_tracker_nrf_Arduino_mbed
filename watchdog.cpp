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
#include "watchdog.h"

void Watchdog::init(uint32_t watchdogtimeout)
{
  //Configure WDT.
  NRF_WDT->CONFIG = 0x01; // Configure WDT to run when CPU is asleep
  // 3932159;  // Timeout set to 120 seconds, timeout[s] = (CRV-1)/32768
  NRF_WDT->CRV = watchdogtimeout; // CRV = timeout * 32768 + 1
  NRF_WDT->RREN = 0x01;           // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;       // Start WDT
}

void Watchdog::reload()
{
  // Reload the WDTs RR[0] reload register
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
