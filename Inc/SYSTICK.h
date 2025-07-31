/*
 * SYSTICK.h
 *
 * Created on: Aug 2, 2024
 * Author: muska
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "stm32f407xx.h"
#include "Def_var.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief  Initializes the SysTick timer with a specified reload value.
 * @param  ticks: Reload value for the SysTick timer. The timer will overflow
 *                 and generate an interrupt when it counts down to zero.
 * @retval None
 */
void SysTick_Init(uint32_t ticks);

/**
 * @brief  SysTick interrupt service routine (ISR).
 * @retval None
 * @note   This function is called by the Cortex-M4 core whenever the SysTick
 *         timer generates an interrupt. It increments the global sysTickCounter.
 */
void SysTick_Handler(void);

/**
 * @brief  Gets the current value of the SysTick counter.
 * @retval Current value of the sysTickCounter.
 */
uint32_t SysTick_GetTicks(void);

/**
 * @brief  Busy-wait delay function based on the SysTick timer.
 * @param  ms: Number of milliseconds to wait.
 * @retval None
 * @note   This function uses a busy-wait loop to wait until the specified number
 *         of milliseconds have elapsed. It is blocking and may not be suitable
 *         for time-critical applications.
 */
void delay_ms(uint32_t ms);

/**
 * @brief  Busy-wait delay function based on the SysTick timer.
 * @param  us: Number of microseconds to wait.
 * @retval None
 * @note   This function uses a busy-wait loop based on the system clock frequency
 *         to wait until the specified number of microseconds have elapsed. It is
 *         blocking and may not be suitable for time-critical applications.
 */
void delay_us(uint32_t us);

/**
 * @brief  Starts a non-blocking delay based on the SysTick timer.
 * @param  durationMs: Duration of the delay in milliseconds.
 * @retval None
 */
void StartNonBlockingDelay(uint32_t durationMs);

/**
 * @brief  Checks if the non-blocking delay has completed.
 * @retval true if delay is complete, false otherwise.
 */
bool IsNonBlockingDelayComplete(void);

#endif /* SYSTICK_H_ */
