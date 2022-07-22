#ifndef __LCD_BIG_SYMBOL_ALERT_H
#define __LCD_BIG_SYMBOL_ALERT_H

#include <Arduino.h>
#include <functional>
#include <hd44780.h>
#include <Print.h>
#include "LcdBigDigits.h"

class LcdBigSymbolAlert
{
  public:
    LcdBigSymbolAlert(hd44780 *lcd, char symbol, uint8_t pos_x) : lcd(lcd), symbol(symbol), pos_x(pos_x)
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

        clear();
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
    hd44780 *lcd;
    char symbol;
    uint8_t pos_x;
    uint16_t timeout = 0, interval = 0;
    unsigned long last_draw = 0, start_time = 0;
    bool active = false, state = false, reset_vals = false;

    void set() {
      showBigDigit(lcd, symbol, pos_x);
    }

    void unset() {
      lcd->setCursor(pos_x, 0);
      lcd->write(":::");

      lcd->setCursor(pos_x, 1);
      lcd->write(":::");
    }

    void clear() {
      lcd->setCursor(pos_x, 0);
      lcd->print("   ");
      
      lcd->setCursor(pos_x, 1);
      lcd->print("   ");
    }
};

#endif
