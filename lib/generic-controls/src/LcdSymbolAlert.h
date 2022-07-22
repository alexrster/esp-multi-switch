#ifndef __LCD_SYMBOL_ALERT_H
#define __LCD_SYMBOL_ALERT_H

#include <Arduino.h>
#include <functional>
#include <Print.h>

class LcdSymbolAlert
{
  public:
    LcdSymbolAlert(char symbol, Print* out) : symbol(symbol), out(out)
    { }

    void blink(uint16_t timeout_ms = 0, uint16_t interval_ms = 600) {
      active = true;
      timeout = timeout_ms;
      interval = interval_ms;
      start_time = millis();
    }

    void reset() {
      if (!active) return;

      reset_vals = true;
    }

    void draw() {
      if (!active) return;

      auto ms = millis();
      if (timeout > 0 && ms - timeout > start_time) {
        reset();
      }

      if (reset_vals) {
        reset_vals = false;
        active = false;

        unset();
        return;
      }

      if (ms - interval > last_draw) {
        last_draw = ms;

        state = !state;
        if (state) set();
        else unset();
      }
    }

  private:
    char symbol;
    Print* out;
    uint16_t timeout = 0, interval = 0;
    unsigned long last_draw = 0, start_time = 0;
    bool active = false, state = false, reset_vals = false;

    void set() {
      out->write(symbol);
    }

    void unset() {
      out->write(' ');
    }
};

#endif
