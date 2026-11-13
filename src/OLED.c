// ****************************************************************
// * Authors: T. LUM and R. MARTIN
// * CPEG222 OLED Demo Program, 9/28/2026
// * NucleoF446ZE CMSIS STM32F4xx example
// * Sends text to the OLED display using SPI
// ****************************************************************
#include "stm32f4xx.h"// CMSIS STM32F4xx header file
#include "OLED.h"
#include <stddef.h>// For size_t definition

#define SPI1_SCK_PIN     5U  // SPI1 SCK pin is connected to PA5
#define SPI1_MISO_PIN    6U  // SPI1 MISO pin is connected to PA6
#define SPI1_MOSI_PIN    7U  // SPI1 MOSI pin is connected to PA7
#define SPI1_CS_PIN      15U  // SPI1 CS pin is connected to PA15 (PA4 is used for analog output)
#define SPI1_PORT        GPIOA  // SPI1 GPIO port
#define OLED_DC_PIN      2U  // OLED DC pin is connected to PG2
#define OLED_RESET_PIN   3U  // OLED RESET pin is connected to PG3
#define OLED_PORT        GPIOG  // OLED GPIO port

static void delay_ms(uint32_t ms);
static void OLED_SetPos(uint8_t page, uint8_t col);
static void OLED_PutChar(uint8_t page, uint8_t col, char c);
static inline void OLED_Select(void) { SPI1_PORT->ODR &= ~(1U << SPI1_CS_PIN); }//macro to select OLED (CS low)
static inline void OLED_Unselect(void) { SPI1_PORT->ODR |= (1U << SPI1_CS_PIN); }//macro to unselect OLED (CS high)
static inline void OLED_DC_Cmd(void) { OLED_PORT->ODR &= ~(1U << OLED_DC_PIN); }//macro to set DC low (command)
static inline void OLED_DC_Data(void) { OLED_PORT->ODR |= (1U << OLED_DC_PIN); }//macro to set DC high (data)
static inline void RES_Low(void) { OLED_PORT->ODR &= ~(1U << OLED_RESET_PIN); }//macro to set RESET low
static inline void RES_High(void) { OLED_PORT->ODR |= (1U << OLED_RESET_PIN); }//macro to set RESET high

static inline void SPI_WriteByte(uint8_t data){ //macro to write a byte via SPI
    while (!(SPI1->SR & SPI_SR_TXE));// Wait until TX buffer is empty
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_TXE));// Wait until TX buffer is empty again
    while (SPI1->SR & SPI_SR_BSY);// Wait until SPI is not busy
}

static void SPI_Write(const uint8_t *buf, size_t len){ //macro to write multiple bytes via SPI
    size_t i;
    for (i = 0; i < len; ++i) {// Loop through each byte in the buffer
        SPI_WriteByte(buf[i]);// Write each byte via SPI
    }
}

 void OLED_WriteCmd(uint8_t cmd){ //macro to write a command to the OLED
    OLED_Select();// Select the OLED (CS low)
    OLED_DC_Cmd();// Set DC low for command
    SPI_WriteByte(cmd);// Write the command byte via SPI
    OLED_Unselect();// Unselect the OLED (CS high)
}

 void OLED_WriteData(const uint8_t *buf, size_t len){ //macro to write data to the OLED
    OLED_Select();// Select the OLED (CS low)
    OLED_DC_Data();// Set DC high for data
    SPI_Write(buf, len);// Write the data bytes via SPI
    OLED_Unselect();// Unselect the OLED (CS high)
}

void OLED_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;// Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;// Enable GPIOG clock

    SPI1_PORT->MODER &= ~((0x3U << (SPI1_SCK_PIN * 2U)) |
                          (0x3U << (SPI1_MISO_PIN * 2U)) |
                          (0x3U << (SPI1_MOSI_PIN * 2U)) |
                          (0x3U << (SPI1_CS_PIN * 2U))); // Clear mode bits for SPI1 pins and CS pin
    SPI1_PORT->MODER |= (0x2U << (SPI1_SCK_PIN * 2U)) |
                        (0x2U << (SPI1_MISO_PIN * 2U)) |
                        (0x2U << (SPI1_MOSI_PIN * 2U)) |
                        (0x1U << (SPI1_CS_PIN * 2U)); // Set SPI1 at AF and CS pin as output

    SPI1_PORT->AFR[0] &= ~((0xFU << (SPI1_SCK_PIN * 4U)) |
                           (0xFU << (SPI1_MISO_PIN * 4U)) |
                           (0xFU << (SPI1_MOSI_PIN * 4U))); // Clear alternate function bits for SPI1 pins
    SPI1_PORT->AFR[0] |= (5U << (SPI1_SCK_PIN * 4U)) |
                         (5U << (SPI1_MISO_PIN * 4U)) |
                         (5U << (SPI1_MOSI_PIN * 4U)); // Set alternate function 5 for SPI1 pins

    SPI1_PORT->OSPEEDR |= (0x3U << (SPI1_SCK_PIN * 2U)) |
                          (0x3U << (SPI1_MOSI_PIN * 2U)); // Set high speed for SPI1 SCK and MOSI pins

    OLED_PORT->MODER &= ~((0x3U << (OLED_DC_PIN * 2U)) |
                          (0x3U << (OLED_RESET_PIN * 2U)));// Clear mode bits for OLED DC and RESET pins
    OLED_PORT->MODER |= (0x1U << (OLED_DC_PIN * 2U)) |
                        (0x1U << (OLED_RESET_PIN * 2U)); // Set OLED DC and RESET pins as output

    OLED_Unselect();
    OLED_DC_Data();
    RES_High();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;// Enable SPI1 clock
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 | SPI_CR1_BR_1;// Configure SPI1: Master, Software slave management, Baud rate
    SPI1->CR2 = 0;// Configure SPI1 control register 2
    SPI1->CR1 |= SPI_CR1_SPE;// Enable SPI1

    RES_Low();
    delay_ms(10);
    RES_High();
    delay_ms(10);

    OLED_WriteCmd(0xAE);// Display off
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);// Set display clock divide ratio/oscillator frequency
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);// Set multiplex ratio
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);// Set display offset
    OLED_WriteCmd(0x40);// Set start line address
    OLED_WriteCmd(0xAD); OLED_WriteCmd(0x8B);// Set DC-DC control mode
    OLED_WriteCmd(0xA1);// Set segment re-map
    OLED_WriteCmd(0xC8);// Set COM output scan direction
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);// Set COM pins hardware configuration
    OLED_WriteCmd(0x81); OLED_WriteCmd(0x7F);// Set contrast control
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0x22);// Set pre-charge period
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x20);// Set VCOMH deselect level
    OLED_WriteCmd(0xA4);// Entire display ON
    OLED_WriteCmd(0xA6);// Set normal display mode
    OLED_WriteCmd(0xAF);// Display ON   

    OLED_Clear();
}

static void OLED_SetPos(uint8_t page, uint8_t col) {// Set the cursor position on the OLED display
    uint8_t x = (uint8_t)(col + 2U);// Calculate the actual column position with an offset of 2
    OLED_WriteCmd((uint8_t)(0xB0U | (page & 0x0FU)));// Set the page address
    OLED_WriteCmd((uint8_t)(0x00U | (x & 0x0FU)));// Set the lower nibble of the column address
    OLED_WriteCmd((uint8_t)(0x10U | ((x >> 4) & 0x0FU)));// Set the upper nibble of the column address
}

static const uint8_t Font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x04,0x08,0x10,0x08}
};

void OLED_Clear(void) {// Clear the display by writing zeros to all pages
    uint8_t zeros[128] = {0};
    uint8_t page;// Page index for iterating through all pages
    for (page = 0; page < 8U; ++page) {// Iterate through all 8 pages
        OLED_SetPos(page, 0);// Set the cursor to the beginning of the current page
        OLED_WriteData(zeros, sizeof(zeros));// Write zeros to the current page
    }
}

static void OLED_PutChar(uint8_t page, uint8_t col, char c) {// Write a single character to the display at the specified page and column
    uint8_t idx;
    uint8_t space = 0x00;

    if ((c < 32) || (c > 126)) {// Replace non-printable characters with '?'
        c = '?';
    }
    idx = (uint8_t)(c - 32);// Calculate the index into the font array
    OLED_SetPos(page, col);// Set the cursor to the specified page and column
    OLED_WriteData(Font5x7[idx], 5);// Write the character data to the display
    OLED_WriteData(&space, 1);// Write a space column after the character
}


void OLED_Print(uint8_t page, uint8_t col, const char *s) {// Print a string to the display starting at the specified page and column
     if (s == NULL) {
        return;
    }
    while (*s != '\0') {
            OLED_PutChar(page, col, *s);// Write the current character to the display
            col = (uint8_t)(col + 6U);// Move the column position for the next character
            if (col > 122U) {// Stop if the column exceeds the display width
                break;
            }
        ++s;// Move to the next character in the string
    }   
}

static void delay_ms(uint32_t ms) {// Simple delay function that loops for the specified number of milliseconds
    uint32_t i;
    for (i = 0; i < (ms * 1000U); ++i) {// Loop for the specified number of iterations to create a delay
        __NOP();// No operation, just waste time
    }
}
