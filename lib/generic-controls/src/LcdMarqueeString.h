#ifndef __LCD_MARQUEE_STRING_H
#define __LCD_MARQUEE_STRING_H

#include <Arduino.h>
#include <Print.h>

class LcdMarqueeString 
{
  public:
    LcdMarqueeString(uint8_t display_len, uint16_t speed_ms = 330) : display_len(display_len), speed_ms(speed_ms)
    {
      sprintf(str_format, "%%%us", display_len);
    }

    void setText(String text)
    {
      if (text == this->text) return;

      if (text.length() > 0)
      {
        this->text = text;
        setInitialOffset();
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

        if (text.length() <= display_len) {
          out->printf("%s%*c", text.c_str(), display_len - text.length(), ' ');       // left-side
          return;
        }

        if (--current_offset <= 0)
        {
          setInitialOffset();
          return;
        }

        int left = current_offset >= text.length() ? 0 : text.length() - current_offset;
        int len = current_offset >= text.length() ? display_len + text.length() - current_offset : display_len;

        String buf = text.substring(left, left + len);
        if (buf.length() < display_len)
        {
          if (current_offset >= text.length()) out->printf(str_format, buf.c_str());  // right-side
          else out->printf("%s%*c", buf.c_str(), display_len - buf.length(), ' ');    // left-side
        }
        else
        {
          out->print(buf.c_str());
        }
      }
    }
  
  private:
    uint8_t display_len = 0;
    uint16_t speed_ms;
    int current_offset = 0;
    long now, last_draw = 0;
    char str_format[5];
    bool text_changed = false;
    String text = emptyString;

    void setInitialOffset()
    {
      if (text.length() > display_len) {
        current_offset = text.length() + display_len;
      }
      else {
        current_offset = 0;
      }

      text_changed = true;
    }
};

#endif