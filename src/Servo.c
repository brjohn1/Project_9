// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 9, 11/12/26
// * Servo file: PWM output (TIM4) and feedback input capture (TIM1)
// *****************************************************************

#include "Servo.h"

// The servos are mounted facing opposite ways, so "forward" is a pulse above 1500 us for one and below for the other.
// If a wheel turns the wrong way on the stand, flip its sign here.
#define LEFT_FWD_SIGN    (+1)   // left servo: forward = pulse above 1500 us
#define RIGHT_FWD_SIGN   (-1)   // right servo: forward = pulse below 1500 us

#define FB_CYCLE_MIN_US  1000   // feedback is 910 Hz (about 1099 us per cycle); ignore anything far from that
#define FB_CYCLE_MAX_US  1200

// Feedback signal state for each servo (index 0 = left, 1 = right)
static uint16_t fbRise[2];          // TIM1 count at the last rising edge
static uint16_t fbCycle[2];         // time between the last two rising edges
static uint8_t  fbHaveRise[2];      // 1 once a rising edge has been seen
static volatile uint16_t fbAngle[2];// angle in tenths of a degree

// Separate speed correction for each servo, in us, added on top of the first-guess pulse offset
static int16_t correction[2];
static uint16_t currentPulse[2] = { SERVO_STOP_US, SERVO_STOP_US };

// Sets up the two PWM outputs: TIM4 CH1 on PD12 (left) and TIM4 CH3 on PD14 (right), both AF2
static void Servo_PWM_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    GPIOD->MODER &= ~((0x3U << (12 * 2)) | (0x3U << (14 * 2)));
    GPIOD->MODER |=  ((0x2U << (12 * 2)) | (0x2U << (14 * 2)));          // alternate function
    GPIOD->AFR[1] &= ~((0xFU << ((12 - 8) * 4)) | (0xFU << ((14 - 8) * 4)));
    GPIOD->AFR[1] |=  ((0x2U << ((12 - 8) * 4)) | (0x2U << ((14 - 8) * 4))); // AF2 = TIM4

    TIM4->PSC = 89;      // TIM4 clock is 90 MHz (2 x 45 MHz APB1), so 90 MHz / 90 = 1 MHz: one tick = 1 us
    TIM4->ARR = 19999;   // 20000 ticks = 20 ms period = 50 Hz, the servo's expected frame rate
    TIM4->CCR1 = SERVO_STOP_US;
    TIM4->CCR3 = SERVO_STOP_US;

    // Channels 1 and 3 in PWM mode 1 (output high while count < CCR) with compare preload
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM4->CCMR2 |=  (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM4->CCER  |=  TIM_CCER_CC1E | TIM_CCER_CC3E;

    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Sets up TIM1 input capture: each feedback pin feeds two channels, one for its rising edge and one for its falling edge
static void Servo_Capture_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PE13 = TIM1_CH3 (left feedback) and PE9 = TIM1_CH1 (right feedback), both AF1
    GPIOE->MODER &= ~((0x3U << (13 * 2)) | (0x3U << (9 * 2)));
    GPIOE->MODER |=  ((0x2U << (13 * 2)) | (0x2U << (9 * 2)));
    GPIOE->AFR[1] &= ~((0xFU << ((13 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOE->AFR[1] |=  ((0x1U << ((13 - 8) * 4)) | (0x1U << ((9 - 8) * 4)));

    TIM1->PSC = 179;     // TIM1 clock is 180 MHz (2 x 90 MHz APB2), so 180 MHz / 180 = 1 MHz: one tick = 1 us
    TIM1->ARR = 0xFFFF;  // free running; uint16_t subtraction handles the wrap

    // Right feedback (PE9 = TI1): CH1 captures TI1 directly (rising), CH2 captures the same pin indirectly (falling)
    TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
    // Left feedback (PE13 = TI3): CH3 captures TI3 directly (rising), CH4 captures the same pin indirectly (falling)
    TIM1->CCMR2 = TIM_CCMR2_CC3S_0 | TIM_CCMR2_CC4S_1;

    // CC1P / CC3P = 0 means rising edge, CC2P / CC4P = 1 means falling edge; enable all four captures
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P |
                 TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC4P;

    TIM1->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
    NVIC_SetPriority(TIM1_CC_IRQn, 0);   // highest priority so edge timestamps are never delayed
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}

void Servo_Init(void) {
    Servo_PWM_Init();
    Servo_Capture_Init();
}

// Rising edge: the time since the last rising edge is one full feedback cycle
void Servo_OnRise(uint8_t servo, uint16_t time) {
    fbCycle[servo] = (uint16_t)(time - fbRise[servo]);
    fbRise[servo] = time;
    fbHaveRise[servo] = 1;
}

// Falling edge: the time since the rising edge is the high time; duty = high / cycle gives the shaft angle
void Servo_OnFall(uint8_t servo, uint16_t time) {
    if (!fbHaveRise[servo]) return;
    uint16_t high  = (uint16_t)(time - fbRise[servo]);
    uint16_t cycle = fbCycle[servo];
    if (cycle < FB_CYCLE_MIN_US || cycle > FB_CYCLE_MAX_US) return;   // ignore a cycle that is not close to 1.1 ms

    uint32_t duty = ((uint32_t)high * 1000U) / cycle;   // tenths of a percent: 29 = 2.9 % = 0 degrees, 971 = 97.1 % = 360 degrees
    if (duty < 29U)  duty = 29U;
    if (duty > 971U) duty = 971U;
    fbAngle[servo] = (uint16_t)(((duty - 29U) * 3600U) / 942U);   // tenths of a degree
}

uint16_t Servo_GetAngle(uint8_t servo) {
    return fbAngle[servo];
}

// Angle change over 100 ms: tenths of a degree per 0.1 s equals degrees per second, and RPM = deg/s / 6
uint16_t Servo_RpmFromAngles(uint16_t previousAngle, uint16_t currentAngle) {
    int32_t d = (int32_t)currentAngle - (int32_t)previousAngle;
    if (d > 1800)  d -= 3600;    // the angle wrapped past 360 degrees going backward
    if (d < -1800) d += 3600;    // the angle wrapped past 360 degrees going forward
    if (d < 0) d = -d;           // speed only, the direction is already known from the buttons
    return (uint16_t)((d + 3) / 6);
}

// First guess for the pulse offset from 1500 us: speed scaled linearly so 140 RPM = 220 us
static int32_t firstGuessOffset(uint16_t targetRpm) {
    return ((int32_t)targetRpm * SERVO_MAX_OFFSET) / SERVO_MAX_RPM;
}

// Closed loop: if the servo is slower than the target the correction pushes the pulse further from 1500 us,
// if faster it moves it closer. The correction is kept separate for each servo.
void Servo_Adjust(uint8_t servo, int8_t dir, uint16_t targetRpm, uint16_t measuredRpm) {
    if (dir == 0 || targetRpm < SERVO_MIN_RPM) {
        correction[servo] = 0;   // stopped, so forget what was learned
        return;
    }
    int32_t error = (int32_t)targetRpm - (int32_t)measuredRpm;
    int32_t c = correction[servo] + (error / 2);   // gain of 1/2 us per RPM of error per step
    if (c >  SERVO_MAX_OFFSET) c =  SERVO_MAX_OFFSET;
    if (c < -SERVO_MAX_OFFSET) c = -SERVO_MAX_OFFSET;
    correction[servo] = (int16_t)c;
}

// Pulse = 1500 us +/- (first guess + correction), with the offset limited to 0 to 220 us
void Servo_Apply(uint8_t servo, int8_t dir, uint16_t targetRpm) {
    int32_t pulse = SERVO_STOP_US;
    if (dir != 0 && targetRpm >= SERVO_MIN_RPM) {
        int32_t offset = firstGuessOffset(targetRpm) + correction[servo];
        if (offset < 0) offset = 0;
        if (offset > SERVO_MAX_OFFSET) offset = SERVO_MAX_OFFSET;
        int32_t sign = (servo == SERVO_LEFT) ? LEFT_FWD_SIGN : RIGHT_FWD_SIGN;
        pulse = SERVO_STOP_US + (int32_t)dir * sign * offset;
    }
    currentPulse[servo] = (uint16_t)pulse;
    if (servo == SERVO_LEFT) {
        TIM4->CCR1 = (uint32_t)pulse;
    } else {
        TIM4->CCR3 = (uint32_t)pulse;
    }
}

uint16_t Servo_GetPulse(uint8_t servo) {
    return currentPulse[servo];
}
