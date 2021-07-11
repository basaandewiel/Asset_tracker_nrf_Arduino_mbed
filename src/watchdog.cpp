#include <Arduino.h>
#include "watchdog.h"

    void Watchdog::init(uint32_t watchdogtimeout)
    {
      //Configure WDT.
      NRF_WDT->CONFIG         = 0x01;     // Configure WDT to run when CPU is asleep
      // 3932159;  // Timeout set to 120 seconds, timeout[s] = (CRV-1)/32768
      NRF_WDT->CRV            = watchdogtimeout;    // CRV = timeout * 32768 + 1
      NRF_WDT->RREN           = 0x01;     // Enable the RR[0] reload register
      NRF_WDT->TASKS_START    = 1;        // Start WDT       
    }

    void Watchdog::reload()
    {
      // Reload the WDTs RR[0] reload register
      NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    }

