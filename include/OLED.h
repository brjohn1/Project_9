#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stddef.h>// For size_t definition
void OLED_Init(void);// Initialize the OLED display
void OLED_Clear(void);// Clear the OLED display
void OLED_WriteCmd(uint8_t cmd);// Write a command to the OLED display
void OLED_WriteData(const uint8_t *buf, size_t len);// Write a buffer of data to the OLED display
void OLED_Print(uint8_t page, uint8_t col, const char *s);// Print a string to the display at the specified page and column
void SH1106_RenderFullScreenLogo(void);// Render the full-screen logo on the OLED display
#endif
