// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 9, 11/12/26
// * NucleoF446ZE + CPEG222 Shield - CMSIS bare metal
// *
// * Closed loop speed control of the robot's two Parallax Feedback 360
// * servos. The pot sets one target speed (0 to 140 RPM), the five buttons
// * choose the direction, TIM1 input capture measures each servo's real
// * speed from its feedback signal, and every 100 ms the PWM pulse is
// * corrected so the real RPM matches the target.
// *   OLED  mode, and each servo's PWM pulse (us) and actual RPM
// *   SSD   target RPM
// *****************************************************************

#include "stm32f4xx.h"
#include "OLED.h"
#include "SysClock180MHz.h"
#include "ssd.h"
#include "Servo.h"
#include <stdint.h>
#include <stdio.h>

/*
 * System Clock: Runs STM32 at 180 MHz.
 * SysTick: Tracks time in 1 ms intervals.
 * TIM1: Measures servo feedback.
 * TIM4: Sends PWM to control servos.
 * TIM5: Refreshes the SSD.
 * ADC3: Reads potentiometer.
 */


#define ENABLE_TANK_TURN   1

// Buttons are active low (pressed = 0) with external pull-ups on the shield
#define BTN_UP_PORT      GPIOF
#define BTN_UP_PIN       7U
#define BTN_DOWN_PORT    GPIOE
#define BTN_DOWN_PIN     5U
#define BTN_LEFT_PORT    GPIOF
#define BTN_LEFT_PIN     9U
#define BTN_RIGHT_PORT   GPIOE
#define BTN_RIGHT_PIN    6U
#define BTN_CENTER_PORT  GPIOF
#define BTN_CENTER_PIN   8U

#define POT_PORT         GPIOC
#define POT_PIN          2U      // V_POT = PC2 (ADC3 channel 12)
#define ADC_CHANNEL      12U
#define ADC_SAMPLES      16U

#define CONTROL_PERIOD_MS  100U  // closed loop period
#define OLED_PERIOD_MS     200U  // OLED redraw period
#define DEBOUNCE_MS        30U   // a button must be released this long before it can register again

typedef enum { BTN_NONE, BTN_CENTER, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT } button_t;
typedef enum { MODE_STOP, MODE_FORWARD, MODE_REVERSE, MODE_LEFT_TURN, MODE_RIGHT_TURN,
               MODE_TANK_LEFT, MODE_TANK_RIGHT } robot_mode_t;

static const char *modeNames[] = { "STOP", "FORWARD", "REVERSE", "LEFT TURN", "RIGHT TURN", "TANK LEFT", "TANK RIGHT" };

// Variables shared between the interrupts and the main loop
volatile uint32_t msTicks = 0;        // counts milliseconds, updated by SysTick
volatile uint8_t  controlFlag = 0;    // set by SysTick every 100 ms
volatile uint16_t snapAngle[2];       // feedback angles copied by SysTick exactly every 100 ms

// Robot state (main loop only)
static robot_mode_t  mode = MODE_STOP;
static int8_t  dirLeft = 0, dirRight = 0;   // +1 forward, -1 reverse, 0 stopped
static uint16_t targetRpm = 0;
static uint16_t rpm[2] = { 0, 0 };          // measured speed of each servo
static uint8_t  btnLock = 0;                // 1 while a button press is being held or still bouncing
static uint32_t lastPressMs = 0;

#if ENABLE_TANK_TURN
static button_t lastTurnButton = BTN_NONE;  // which turn button was pressed last
static uint8_t  turnCount = 0;              // consecutive presses of that button
#endif

// Sets the five buttons up as plain digital inputs
static void Buttons_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN;
    BTN_UP_PORT->MODER     &= ~(0x3U << (BTN_UP_PIN * 2U));
    BTN_DOWN_PORT->MODER   &= ~(0x3U << (BTN_DOWN_PIN * 2U));
    BTN_LEFT_PORT->MODER   &= ~(0x3U << (BTN_LEFT_PIN * 2U));
    BTN_RIGHT_PORT->MODER  &= ~(0x3U << (BTN_RIGHT_PIN * 2U));
    BTN_CENTER_PORT->MODER &= ~(0x3U << (BTN_CENTER_PIN * 2U));
}

// Sets up ADC3 channel 12 (PC2) for reading the potentiometer
static void Pot_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    POT_PORT->MODER |= (0x3U << (POT_PIN * 2U));   // analog mode

    // ADC clock = PCLK2 / prescaler. PCLK2 is 90 MHz at 180 MHz, so the default /2 gives 45 MHz, above the 36 MHz maximum.
    // ADCPRE (bits 17:16 of the ADC common control register at 0x40012304) = 01 selects /4 = 22.5 MHz.
    volatile uint32_t *adcCommonCCR = (volatile uint32_t *)0x40012304UL;
    *adcCommonCCR = (*adcCommonCCR & ~(0x3UL << 16)) | (0x1UL << 16);

    ADC3->SQR3  = ADC_CHANNEL;
    ADC3->SMPR1 = ADC_SMPR1_SMP12_0 | ADC_SMPR1_SMP12_1;   // 56 cycles per sample
    ADC3->CR2   = ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 20000U; i++) { }      // let the ADC power up
}

// Takes 16 ADC readings and averages them to reduce noise; returns 0 to 4095
static uint16_t Pot_Read(void) {
    uint32_t total = 0;
    for (uint32_t i = 0; i < ADC_SAMPLES; i++) {
        ADC3->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC3->SR & ADC_SR_EOC)) { }
        total += ADC3->DR;
    }
    return (uint16_t)(total / ADC_SAMPLES);
}

// Configures SysTick to interrupt every 1 ms (SysTick alone cannot count 100 ms, its limit is about 93 ms)
static void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 1);
}

// Configures TIM5 to interrupt at 400 Hz (every 2.5 ms) so each of the 4 SSD digits refreshes at 100 Hz
static void TIM5_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIM5->PSC = 8999;  // TIM5 clock is 90 MHz (2 x 45 MHz APB1), so 90 MHz / 9000 = 10 kHz -> 0.1 ms per tick
    TIM5->ARR = 24;    // 25 ticks = 2.5 ms period -> 400 Hz
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR  = ~TIM_SR_UIF;
    TIM5->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->CR1 = TIM_CR1_CEN;
}

// Every 1 ms: counts time, and every 100 ms copies both feedback angles so the speed is measured over exactly 100 ms
void SysTick_Handler(void) {
    static uint32_t periodCount = 0;
    msTicks++;
    if (++periodCount >= CONTROL_PERIOD_MS) {
        periodCount = 0;
        snapAngle[SERVO_LEFT]  = Servo_GetAngle(SERVO_LEFT);
        snapAngle[SERVO_RIGHT] = Servo_GetAngle(SERVO_RIGHT);
        controlFlag = 1;
    }
}

// Every 2.5 ms: refreshes one SSD digit
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR = ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

// TIM1 capture interrupt: each flag means one feedback edge was captured (reading the CCR value also clears the flag)
void TIM1_CC_IRQHandler(void) {
    uint32_t sr = TIM1->SR;
    if (sr & TIM_SR_CC1IF) Servo_OnRise(SERVO_RIGHT, (uint16_t)TIM1->CCR1);   // right feedback rising
    if (sr & TIM_SR_CC2IF) Servo_OnFall(SERVO_RIGHT, (uint16_t)TIM1->CCR2);   // right feedback falling
    if (sr & TIM_SR_CC3IF) Servo_OnRise(SERVO_LEFT,  (uint16_t)TIM1->CCR3);   // left feedback rising
    if (sr & TIM_SR_CC4IF) Servo_OnFall(SERVO_LEFT,  (uint16_t)TIM1->CCR4);   // left feedback falling
}

// Reads the five buttons (pressed = 0); if several are held, the first one in this list wins
static button_t readButtons(void) {
    if (!(BTN_CENTER_PORT->IDR & (1U << BTN_CENTER_PIN))) return BTN_CENTER;
    if (!(BTN_UP_PORT->IDR     & (1U << BTN_UP_PIN)))     return BTN_UP;
    if (!(BTN_DOWN_PORT->IDR   & (1U << BTN_DOWN_PIN)))   return BTN_DOWN;
    if (!(BTN_LEFT_PORT->IDR   & (1U << BTN_LEFT_PIN)))   return BTN_LEFT;
    if (!(BTN_RIGHT_PORT->IDR  & (1U << BTN_RIGHT_PIN)))  return BTN_RIGHT;
    return BTN_NONE;
}

// Pushes the current direction and target out to both servos right now
static void applyServos(void) {
    Servo_Apply(SERVO_LEFT,  dirLeft,  targetRpm);
    Servo_Apply(SERVO_RIGHT, dirRight, targetRpm);
}

// Sets the mode and each servo's direction for one button press, then applies it immediately
static void handleButton(button_t b) {
#if ENABLE_TANK_TURN
    // Extra credit: a second consecutive LEFT or RIGHT press tank turns; a third is ignored; any other button resets the count
    if (b == BTN_LEFT || b == BTN_RIGHT) {
        if (b == lastTurnButton) {
            if (turnCount >= 2) return;   // third press in a row: ignore
            turnCount++;
        } else {
            lastTurnButton = b;
            turnCount = 1;
        }
    } else {
        lastTurnButton = BTN_NONE;
        turnCount = 0;
    }
#endif

    switch (b) {
        case BTN_CENTER: mode = MODE_STOP;    dirLeft =  0; dirRight =  0; break;
        case BTN_UP:     mode = MODE_FORWARD; dirLeft = +1; dirRight = +1; break;
        case BTN_DOWN:   mode = MODE_REVERSE; dirLeft = -1; dirRight = -1; break;
        case BTN_LEFT:
            mode = MODE_LEFT_TURN;  dirLeft = 0;  dirRight = +1;           // left servo stopped, right servo forward
#if ENABLE_TANK_TURN
            if (turnCount == 2) { mode = MODE_TANK_LEFT; dirLeft = -1; }   // left servo reverses
#endif
            break;
        case BTN_RIGHT:
            mode = MODE_RIGHT_TURN; dirLeft = +1; dirRight = 0;            // right servo stopped, left servo forward
#if ENABLE_TANK_TURN
            if (turnCount == 2) { mode = MODE_TANK_RIGHT; dirRight = -1; } // right servo reverses
#endif
            break;
        default: return;
    }
    applyServos();
}

// Polls the buttons with the lock technique: a press registers once, then is locked out until
// every button has been released for DEBOUNCE_MS, which also ignores contact bounce
static void pollButtons(void) {
    button_t b = readButtons();
    if (b != BTN_NONE) {
        lastPressMs = msTicks;
        if (!btnLock) {
            btnLock = 1;
            handleButton(b);
        }
    } else if (btnLock && (msTicks - lastPressMs) >= DEBOUNCE_MS) {
        btnLock = 0;
    }
}

// Runs every 100 ms: reads the pot, measures each servo's speed, corrects the pulses, and updates the SSD
static void controlStep(void) {
    static uint8_t havePrev = 0;
    static uint16_t prevAngle[2];

    // Pot 0 to 4095 -> target 0 to 140 RPM; below 5 RPM counts as stop
    uint32_t pot = Pot_Read();
    targetRpm = (uint16_t)((pot * SERVO_MAX_RPM + 2047U) / 4095U);
    if (targetRpm < SERVO_MIN_RPM) targetRpm = 0;

    // Measured speed from the angle change over exactly 100 ms (the first pass only records the angles)
    if (havePrev) {
        rpm[SERVO_LEFT]  = Servo_RpmFromAngles(prevAngle[SERVO_LEFT],  snapAngle[SERVO_LEFT]);
        rpm[SERVO_RIGHT] = Servo_RpmFromAngles(prevAngle[SERVO_RIGHT], snapAngle[SERVO_RIGHT]);
    }
    prevAngle[SERVO_LEFT]  = snapAngle[SERVO_LEFT];
    prevAngle[SERVO_RIGHT] = snapAngle[SERVO_RIGHT];
    havePrev = 1;

    Servo_Adjust(SERVO_LEFT,  dirLeft,  targetRpm, rpm[SERVO_LEFT]);
    Servo_Adjust(SERVO_RIGHT, dirRight, targetRpm, rpm[SERVO_RIGHT]);
    applyServos();

    SSD_SetValueInt(targetRpm);   // both servos share one target, so the SSD does not depend on the buttons or feedback
}

// Prints a line on the OLED padded to 21 characters so leftovers from the previous reading are erased
static void showLine(uint8_t row, const char *text) {
    char padded[24];
    snprintf(padded, sizeof(padded), "%-21s", text);
    OLED_Print(row, 0, padded);
}

// Redraws the mode and one line per servo with its actual PWM pulse (us) and actual RPM
static void updateOled(void) {
    char line[32];
    snprintf(line, sizeof(line), "Mode: %s", modeNames[mode]);
    showLine(2, line);
    snprintf(line, sizeof(line), "L PWM %4u RPM %3u", (unsigned)Servo_GetPulse(SERVO_LEFT), (unsigned)rpm[SERVO_LEFT]);
    showLine(3, line);
    snprintf(line, sizeof(line), "R PWM %4u RPM %3u", (unsigned)Servo_GetPulse(SERVO_RIGHT), (unsigned)rpm[SERVO_RIGHT]);
    showLine(4, line);
    snprintf(line, sizeof(line), "Target: %3u RPM", (unsigned)targetRpm);
    showLine(5, line);
}

int main(void) {
    // Same clock setup as Project 5: 16 MHz at reset, then 180 MHz
    SystemCoreClockUpdate();
    SystemClock_Config();
    SystemCoreClockUpdate();

    // Peripherals are set up after the clock change so every timer and ADC value is correct
    OLED_Init();
    OLED_Clear();
    OLED_Print(0, 0, "CPEG222-F26 Project 9");
    OLED_Print(1, 0, "Brian Johnson");

    Buttons_Init();
    Pot_Init();
    SSD_Init();
    SSD_SetDecimalPoint(SSD_NO_DP);   // the target RPM is a plain integer
    SSD_SetValueInt(0);
    TIM5_Init();
    Servo_Init();                     // starts both servos at the 1500 us stop pulse

    applyServos();                    // the robot starts stopped at reset
    updateOled();
    SysTick_Init();                   // start last so the first control step comes 100 ms after everything is ready

    uint32_t lastOledMs = 0;

    while (1) {
        pollButtons();

        if (controlFlag) {
            controlFlag = 0;
            controlStep();
        }

        if ((msTicks - lastOledMs) >= OLED_PERIOD_MS) {
            lastOledMs = msTicks;
            updateOled();
        }
    }
    return 0;
}
