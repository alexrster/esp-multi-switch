#include <Arduino.h>
#include "LcdBigDigits.h"

#define CH_I(x) (BIG_DIGIT_FIRST_CHAR_INDEX+x)

const PROGMEM uint8_t glyphs[5][8]={
  {B11111,B11111,B00000,B00000,B00000,B00000,B00000,B00000},
  {B00000,B00000,B00000,B00000,B00000,B00000,B11111,B11111},
  {B11111,B11111,B00000,B00000,B00000,B00000,B11111,B11111},
  {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111},
  {B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110}
};

const int digitWidth = 3;
const char bigDigitsTop[12][digitWidth]={CH_I(3),CH_I(0),CH_I(3), CH_I(0),CH_I(3),32,       CH_I(2),CH_I(2),CH_I(3), CH_I(0),CH_I(2),CH_I(3), CH_I(3),CH_I(1),CH_I(3),  CH_I(3),CH_I(2),CH_I(2), CH_I(3),CH_I(2),CH_I(2), CH_I(0),CH_I(0),CH_I(3),   CH_I(3),CH_I(2),CH_I(3),   CH_I(3),CH_I(2),CH_I(3),  CH_I(3),CH_I(1),CH_I(3),  CH_I(3),CH_I(2),CH_I(0)};
const char bigDigitsBot[12][digitWidth]={CH_I(3),CH_I(1),CH_I(3), CH_I(1),CH_I(3),CH_I(1),  CH_I(3),CH_I(1),CH_I(1), CH_I(1),CH_I(1),CH_I(3), 32,32,CH_I(3),            CH_I(1),CH_I(1),CH_I(3), CH_I(3),CH_I(1),CH_I(3), 32,32,CH_I(3),             CH_I(3),CH_I(1),CH_I(3),   CH_I(1),CH_I(1),CH_I(3),  CH_I(3),32,CH_I(3),       CH_I(3),CH_I(1),CH_I(1)};

char buffer[12];  //used to convert a number into a string

void setupBigDigit(hd44780 *lcd)
{
  for(int i=0; i<4; i++)
    lcd->createChar(BIG_DIGIT_FIRST_CHAR_INDEX+i, glyphs[i]);  //create the 5 custom glyphs
}

void showBigDigit(hd44780 *lcd, int digit, int position)
{
  lcd->setCursor(position, 0);
  for(int i = 0; i < digitWidth; i++)
    lcd->write(bigDigitsTop[digit][i]);

  lcd->setCursor(position, 1);
  for(int i = 0; i < digitWidth; i++)
    lcd->write(bigDigitsBot[digit][i]);  
}

void showBigNumber(hd44780 *lcd, char* value, int position)
{
  // setupBigDigit(lcd);
  for(int index = 0; index < 12; index++)
  {
    char c = value[index];
    
    if(c == 0)
      return;
    
    c -= 48;
    showBigDigit(lcd, c, position + index * (digitWidth + 1));
  }  
}

void showBigNumber(hd44780 *lcd, int value, int position)
{
  itoa(value, buffer, 10);
  showBigNumber(lcd, buffer, position);
}

void showBigNumberFixed(hd44780 *lcd, int value, byte minLength, int position)
{
  char format[8];
  sprintf(format, "%%0%dd", minLength);
  sprintf(buffer, format, value);
  showBigNumber(lcd, buffer, position);
}
