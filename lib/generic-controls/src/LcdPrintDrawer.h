#ifndef __LCD_DRAWER_H
#define __LCD_DRAWER_H

#include <Print.h>

class LcdPrintDrawer : public Print
{
  public:
    LcdPrintDrawer(Print *out) : printOut(out)
    { }

    virtual size_t write(uint8_t v)
    {
      text.clear();
      text.concat(v);
      return text.length();
    }

    virtual size_t write(const uint8_t *buffer, size_t size)
    {
      text.clear();
      text.concat((const char*)buffer, size);
      return text.length();
    }

    void draw()
    {
      printOut->print(text.c_str());
    }
  
  private:
    Print *printOut;
    String text;
};

#endif