#ifndef __LCD_BLINK_STRING_H
#define __LCD_BLINK_STRING_H

#include <Arduino.h>
#include <Print.h>

class LcdBlinkString 
{
  public:
    LcdBlinkString(uint8_t display_len, uint16_t speed_ms = 1000) : display_len(display_len), speed_ms(speed_ms)
    { }

    void setText(String text)
    {
      if (this->text.equals(text)) return;

      if (text.length() > 0)
      {
        this->text = text.c_str();
        current_offset = 0;
        text_changed = true;
      }
      else
      {
        this->text = emptyString;
        current_offset = 0;
      }

      text_changed = true;
    }

    void draw(Print* out)
    {
      draw(out, false);
    }

    void draw(Print* out, bool force)
    {
      if (!force && !text_changed && text.length() <= 0) return;

      now = millis();
      if (force || text_changed || now - last_draw >= speed_ms) {
        last_draw = now;
        text_changed = false;

        if (text.length() == 0) clearOut(out);
        else printOut(out);
      }
    }
  
  private:
    uint8_t display_len = 0;
    uint16_t speed_ms;
    int current_offset = 0;
    long now, last_draw = 0;
    bool text_changed = false;
    String text = emptyString;

    inline void clearOut(Print* out) 
    {
      for (uint8_t i = 0; i < display_len; i++)
        out->print(' ');
    }

    inline size_t printOut(Print* out)
    {
      if (text.length() <= display_len) {
        if (text.length() < display_len) clearOut(out);
        return out->write(text.c_str(), text.length());
      }
      else if (current_offset + display_len >= text.length()) {
        clearOut(out);
        auto ret = out->write(text.substring(current_offset, text.length()).c_str(), text.length() - current_offset);
        current_offset = 0;
        return ret;
      }
      else {
        auto ret = out->write(text.substring(current_offset, current_offset + display_len).c_str(), display_len);
        current_offset += display_len;
        return ret;
      }

      return 0;
    }
};

#endif