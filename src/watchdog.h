#ifndef NRF_WATCHDOG_H
#define NRF_WATCHDOG_H

class Watchdog
{
private:
public:
  void init(uint32_t watchdogtimeout);
  void reload();
};

#endif /* NRF_WATCHDOG_H*/