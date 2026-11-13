// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * seven segment display (SSD) file
// *****************************************************************

#ifndef SSD_H
#define SSD_H

#include "stm32f4xx.h"

#define SSD_NO_DP 0xFF  // pass to SSD_SetDecimalPoint() to turn the decimal point off entirely

// Sets up all GPIO pins used by the segments and digit enables, and blanks the display
void SSD_Init(void);

// Project 3 format: converts a 0-9999 hundredths-of-a-second value into the 4 digits to display
void SSD_SetValue(uint16_t hundredths);

// Project 7 format: displays a frequency given in tenths of a Hz (xxx.x), blanking leading zeros but always showing the ones digit
void SSD_SetValueTenths(uint16_t tenths);

// Project 4 format: displays a plain 0-9999 integer across the 4 digits, blanking a leading zero
void SSD_SetValueInt(uint16_t value);

// Chooses which digit (0-3) shows the decimal point, or SSD_NO_DP for none; defaults to digit 1
void SSD_SetDecimalPoint(uint8_t digitIndex);

// Lights exactly one digit and advances to the next; call this every interrupt from TIM5's ISR
void SSD_Refresh(void);

#endif