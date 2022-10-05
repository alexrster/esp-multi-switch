#ifndef BLINDS_CONTROLLER_H
#define BLINDS_CONTROLLER_H

#include <Arduino.h>
#include "SwitchRelay.h"

class BlindsController {
  public:
    BlindsController(SwitchRelay motorUp, SwitchRelay motorDown, uint8_t zeroTopPin)
      : motorUp(motorUp), motorDown(motorDown), zeroTopPin(zeroTopPin)
    { 
      pinMode(zeroTopPin, INPUT_PULLUP);
      // attachInterrupt(zeroTopPin, , FALLING);
    }
  
  private:
    SwitchRelay motorUp;
    SwitchRelay motorDown;
    uint8_t zeroTopPin;
};

#endif