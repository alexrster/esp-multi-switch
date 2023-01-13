#ifndef __LCD_DRAWER_H
#define __LCD_DRAWER_H

#include <Print.h>

class LcdPrintDrawer : public Print
{
  public:
    LcdPrintDrawer(uint16_t maxLength = 4) : maxLength(maxLength)
    {
      text.reserve(maxLength);
    }

    virtual size_t write(uint8_t v)
    {
      text = v;

      redraw = true;
      return text.length();
    }

    virtual size_t write(const uint8_t *buffer, size_t size)
    {
      text = (const char*)buffer;

      redraw = true;
      return text.length();
    }

    void draw(Print *out)
    {
      if (!redraw) return;
      redraw = false;

      out->write(text.c_str(), text.length() >= maxLength ? maxLength : text.length());
    }
  
  private:
    uint16_t maxLength;
    String text;
    bool redraw = false;
};

#endif