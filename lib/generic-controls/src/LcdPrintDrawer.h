#ifndef __LCD_DRAWER_H
#define __LCD_DRAWER_H

#include <Print.h>

class LcdPrintDrawer : public Print
{
  public:
    LcdPrintDrawer(Print *out, uint16_t maxLength = 4) : printOut(out), maxLength(maxLength)
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

    void draw()
    {
      if (!redraw) return;
      redraw = false;

      printOut->write(text.c_str(), text.length() >= maxLength ? maxLength : text.length());
    }
  
  private:
    Print *printOut;
    uint16_t maxLength;
    String text;
    bool redraw = false;
};

#endif