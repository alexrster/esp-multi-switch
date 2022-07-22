#ifndef __LCD_BIG_DIGITS_H
#define __LCD_BIG_DIGITS_H

#include <hd44780.h>

#define BIG_DIGIT_FIRST_CHAR_INDEX     4

void setupBigDigit(hd44780 *lcd);
void showBigDigit(hd44780 *lcd, int digit, int position);
void showBigNumberFixed(hd44780 *lcd, int value, byte minLength, int position);

#endif