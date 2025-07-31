/*
 * Def_var.h
 *
 *  Created on: Aug 21, 2024
 *      Author: Muskaan Jain
 */

#ifndef DEF_VAR_H_
#define DEF_VAR_H_

/* Macros defined for target application */
#define SWITCH_PIN      0    // Define according to your switch pin
#define SWITCH_PORT     GPIOA // Define according to your switch port
#define LED_PIN         12   // LED connected to PD12
#define LED_PORT        GPIOD // LED connected to Port D
#define LED_OFF         GPIO_PIN_RESET
#define DEBOUNCE_DELAY  50   // Debounce delay in milliseconds
#define SystemCoreClock 168000000
#define LED_PATTERNS_COUNT  4

/* LED PATTERN DRIVER */
// Timing variables
extern volatile uint32_t patternStartTime;  // Declare the variable
extern volatile uint32_t patternDelay;

/* SYSTICK DRIVER */
extern volatile uint32_t sysTickCounter;    // Declare the variable
extern volatile uint32_t delayStartTime;
extern volatile uint32_t delayDuration;

/* MAIN FILE */
extern volatile uint32_t current_blink_period;  // Declare the variable
extern volatile uint8_t pressCount;
extern volatile uint32_t last_interrupt_time;
extern const uint32_t blink_periods[LED_PATTERNS_COUNT];  // Array of blink periods

#endif /* DEF_VAR_H_ */
