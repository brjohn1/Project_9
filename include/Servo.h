// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 9, 11/12/26
// * Servo file: PWM output (TIM4) and feedback input capture (TIM1)
// * for the Parallax Feedback 360 continuous rotation servos
// *****************************************************************

#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx.h"

#define SERVO_LEFT   0
#define SERVO_RIGHT  1

#define SERVO_STOP_US    1500   // pulse width that stops a servo
#define SERVO_MAX_OFFSET 220    // pulse offset from 1500 us that gives full speed (about 140 RPM)
#define SERVO_MAX_RPM    140
#define SERVO_MIN_RPM    5      // a target below this is treated as stop

// Sets up PD12/PD14 as TIM4 PWM outputs (50 Hz, 1 us steps) and PE13/PE9 as TIM1 input capture pins (1 MHz free-running)
void Servo_Init(void);

// Called from the TIM1 capture interrupt with the TIM1 count when a feedback pin rose or fell
void Servo_OnRise(uint8_t servo, uint16_t time);
void Servo_OnFall(uint8_t servo, uint16_t time);

// Most recent shaft angle from the feedback signal, in tenths of a degree (0 to 3599)
uint16_t Servo_GetAngle(uint8_t servo);

// Speed in RPM from two angles taken exactly 100 ms apart (handles the 360 degree wrap)
uint16_t Servo_RpmFromAngles(uint16_t previousAngle, uint16_t currentAngle);

// Closed loop step (call every 100 ms): nudges this servo's correction toward the target speed
// dir is +1 forward, -1 reverse, 0 stopped (all from the robot's point of view)
void Servo_Adjust(uint8_t servo, int8_t dir, uint16_t targetRpm, uint16_t measuredRpm);

// Writes the PWM pulse for this servo from the direction, the target speed and its stored correction
void Servo_Apply(uint8_t servo, int8_t dir, uint16_t targetRpm);

// The pulse width (in us) the servo is being sent right now
uint16_t Servo_GetPulse(uint8_t servo);

#endif
