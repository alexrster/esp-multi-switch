#ifndef __LCD_FIXED_POSITION_PRINT_H
#define __LCD_FIXED_POSITION_PRINT_H

#include <hd44780.h>
#include <Print.h>

class LcdFixedPositionPrint : public Print
{
  public:
    LcdFixedPositionPrint(hd44780* lcd, uint8_t row=0, uint8_t col=0) : Print(), lcd(lcd), row(row), col(col)
    { }

    virtual size_t write(uint8_t v)
    {
      lcd->setCursor(col, row);
      return lcd->write(v);
    }

    virtual size_t write(const uint8_t *buffer, size_t size)
    {
      lcd->setCursor(col, row);
      return lcd->write((const char*)buffer, size);
    }
  
  private:
    hd44780* lcd;
    uint8_t row;
    uint8_t col;
};

#endif
