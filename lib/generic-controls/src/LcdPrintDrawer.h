#ifndef __LCD_DRAWER_H
#define __LCD_DRAWER_H

#include <Print.h>

class LcdPrintDrawer : public Print
{
  public:
    LcdPrintDrawer(Print *out, uint16_t bufSize = 4) : printOut(out)
    {
      text.reserve(bufSize);
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

      printOut->write((const uint8_t*)text.c_str(), (size_t)text.length());
    }
  
  private:
    Print *printOut;
    String text;
    bool redraw = false;
};

#endif