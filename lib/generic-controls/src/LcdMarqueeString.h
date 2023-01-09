#ifndef __LCD_MARQUEE_STRING_H
#define __LCD_MARQUEE_STRING_H

#include <Arduino.h>
#include <Print.h>

class LcdMarqueeString 
{
  public:
    LcdMarqueeString(uint8_t display_len, uint16_t speed_ms = 330) : display_len(display_len), speed_ms(speed_ms)
    {
      sprintf(str_format_right, "%%%us", display_len);
      sprintf(str_format_left, "%%-%us", display_len);

      textRenderBuffer = (char *)malloc(sizeof(char) * display_len);
    }

    void setText(String text)
    {
      if (this->text.equals(text)) return;

      if (text.length() > 0)
      {
        this->text = text.c_str();
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

        if (text.length() == 0) {
          writeTextRenderBuffer(out, snprintf(textRenderBuffer, display_len, str_format_left, " "));
          return;
        }

        if (text.length() <= display_len) {
          writeTextRenderBuffer(out, snprintf(textRenderBuffer, display_len, str_format_left, text.c_str()));
          return;
        }

        if (--current_offset <= 0)
        {
          setInitialOffset();
          return;
        }

        int left = current_offset >= text.length() ? 0 : text.length() - current_offset;
        int len = current_offset >= text.length() ? display_len + text.length() - current_offset : display_len;

        String buf = text.substring(left, left + len - 1);
        if (buf.length() < display_len)
        {
          if (current_offset >= text.length()) {    // right-side
            writeTextRenderBuffer(out, snprintf(textRenderBuffer, display_len, str_format_right, buf.c_str()));
          }
          else {                                    // left-side
            writeTextRenderBuffer(out, snprintf(textRenderBuffer, display_len, str_format_left, buf.c_str()));
          }
        }
        else
        {
          out->write(buf.c_str(), display_len);
        }
      }
    }
  
  private:
    uint8_t display_len = 0;
    uint16_t speed_ms;
    int current_offset = 0;
    long now, last_draw = 0;
    bool text_changed = false;
    String text = emptyString;
    char *textRenderBuffer;
    char str_format_left[5] = {0,0,0,0,0};
    char str_format_right[4] = {0,0,0,0};

    inline size_t writeTextRenderBuffer(Print* out, int n)
    {
      n = n < 0 ? 0 : n > display_len ? display_len : n;
      if (n > 0) {
#ifdef DEBUG
        auto writtenBytes = out->write(textRenderBuffer, n);
        if (writtenBytes != n) {
          log_w("Potential DATA LOSS during writing to Printable sink! sent=%u, written=%u", n, writtenBytes);
        }

        return writtenBytes;
#else
        return out->write(textRenderBuffer, n);
#endif
      }

      return 0;
    }

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