// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 8, 11/5/26
// * seven segment display (SSD) file
// *****************************************************************

#include "ssd.h"

typedef struct { GPIO_TypeDef *port; uint8_t pin; } PinDef;

// Segment pins in order A, B, C, D, E, F, G, DP
static const PinDef segPins[8] = {
    {GPIOG, 9},   // A
    {GPIOF, 12},  // B
    {GPIOF, 13},  // C
    {GPIOG, 14},  // D
    {GPIOE, 8},   // E
    {GPIOF, 15},  // F
    {GPIOB, 4},   // G
    {GPIOF, 14}   // DP
};

// Digit-enable pins in order: thousands/tens, hundreds/ones, tens/tenths, ones/hundredths
static const PinDef digitPins[4] = {
    {GPIOE, 10},  // CA1
    {GPIOE, 7},   // CA2
    {GPIOB, 5},   // CA3
    {GPIOB, 3}    // CA4
};

#define SSD_BLANK 10

// Logical segment patterns for 0-9 and blank; bit i = 1 means "segment i is ON"
static const uint8_t segmentTable[11] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000  // blank
};

// Holds the digit currently being shown, which digit multiplexing is on, and the decimal point setting
static volatile uint8_t digitValues[4] = { SSD_BLANK, 0, 0, 0 };
static volatile uint8_t currentDigit = 0;
static volatile uint8_t decimalDigit = 1; // defaults to Project 3's original behavior

// Drives one pin HIGH or LOW using BSRR for an atomic write
static inline void setPin(GPIO_TypeDef *port, uint8_t pin, uint8_t level) {
    if (level) {
        port->BSRR = (1 << pin);
    } else {
        port->BSRR = (1 << (pin + 16));
    }
}

// Turns every segment and every digit off, used between multiplex steps to prevent ghosting
static void SSD_AllOff(void) {
    for (int i = 0; i < 8; i++) {
        setPin(segPins[i].port, segPins[i].pin, 1);
    }
    for (int i = 0; i < 4; i++) {
        setPin(digitPins[i].port, digitPins[i].pin, 1);
    }
}

// Enables clocks, sets all segment/digit pins as outputs, and blanks the display
void SSD_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN
                  | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;

    for (int i = 0; i < 8; i++) {
        GPIO_TypeDef *p = segPins[i].port;
        uint8_t pin = segPins[i].pin;
        p->MODER &= ~(0x3 << (pin * 2));
        p->MODER |=  (0x1 << (pin * 2));
    }
    for (int i = 0; i < 4; i++) {
        GPIO_TypeDef *p = digitPins[i].port;
        uint8_t pin = digitPins[i].pin;
        p->MODER &= ~(0x3 << (pin * 2));
        p->MODER |=  (0x1 << (pin * 2));
    }

    SSD_AllOff();
}

// Splits a 0-9999 hundredths value into 4 digits (Project 3's stopwatch format)
void SSD_SetValue(uint16_t hundredths) {
    if (hundredths > 9999) hundredths = 9999;

    uint8_t tens    = hundredths / 1000;
    uint8_t ones    = (hundredths / 100) % 10;
    uint8_t tenths  = (hundredths / 10) % 10;
    uint8_t hund    = hundredths % 10;

    digitValues[0] = (tens == 0) ? SSD_BLANK : tens;
    digitValues[1] = ones;
    digitValues[2] = tenths;
    digitValues[3] = hund;
}

// Splits a plain 0-9999 integer into 4 digits, blanking only a leading-zero thousands digit
void SSD_SetValueInt(uint16_t value) {
    if (value > 9999) value = 9999;

    uint8_t thousands = value / 1000;
    uint8_t hundreds  = (value / 100) % 10;
    uint8_t tens      = (value / 10) % 10;
    uint8_t ones      = value % 10;

    digitValues[0] = (thousands == 0) ? SSD_BLANK : thousands;
    digitValues[1] = hundreds;
    digitValues[2] = tens;
    digitValues[3] = ones;
}

// Splits a tenths-of-a-Hz value (xxx.x) into 4 digits, blanking leading zeros but keeping the ones digit so 0 shows as 0.0
void SSD_SetValueTenths(uint16_t tenths) {
    if (tenths > 9999) tenths = 9999;

    uint8_t hundreds = tenths / 1000;
    uint8_t tens     = (tenths / 100) % 10;
    uint8_t ones     = (tenths / 10) % 10;
    uint8_t tenth    = tenths % 10;

    digitValues[0] = (hundreds == 0) ? SSD_BLANK : hundreds;
    digitValues[1] = (hundreds == 0 && tens == 0) ? SSD_BLANK : tens;
    digitValues[2] = ones;
    digitValues[3] = tenth;
}

// Sets which digit shows the decimal point (or disables it with SSD_NO_DP)
void SSD_SetDecimalPoint(uint8_t digitIndex) {
    decimalDigit = digitIndex;
}

// Multiplex step: blanks everything, lights the current digit's segments, then moves to the next digit
void SSD_Refresh(void) {
    SSD_AllOff();

    uint8_t value   = digitValues[currentDigit];
    uint8_t pattern = segmentTable[value];

    for (int i = 0; i < 7; i++) {
        uint8_t on = (pattern >> i) & 0x1;
        setPin(segPins[i].port, segPins[i].pin, on ? 0 : 1); // active-low segment
    }

    uint8_t dpOn = (currentDigit == decimalDigit);
    setPin(segPins[7].port, segPins[7].pin, dpOn ? 0 : 1);

    setPin(digitPins[currentDigit].port, digitPins[currentDigit].pin, 0); // enable this digit

    currentDigit = (currentDigit + 1) % 4;
}