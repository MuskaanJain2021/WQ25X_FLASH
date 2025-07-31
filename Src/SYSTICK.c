#include "stm32f4xx.h"
//Total cycles = LOAD + 1 = ticks or No of cycles that needs to be loaded in reload (sysTick->load or reload register)
//Time taken = Cycles / System Clock
//SysTick is 24-bit: Max LOAD = 16,777,215
//At 16MHz:

//Max ms delay: ~1048ms (16,777,215 / 16,000)

//Max μs delay: ~1,048,575μs (16,777,215 / 16)

volatile uint32_t msTicks = 0;
volatile uint32_t usTicks = 0;
volatile uint8_t delay_mode = 0; // 0 = none, 1 = ms, 2 = us
uint32_t System_CoreClock = 16000000;

void SysTick_Enable(uint32_t ticks, uint8_t mode) {
	delay_mode = mode;
	SysTick->LOAD = ticks - 1; // Set reload value ,LOAD=(System_CoreClock×Desired Delay (s))−1
	SysTick->VAL = 0; // Reset counter or Clear current value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
	SysTick_CTRL_TICKINT_Msk |
	SysTick_CTRL_ENABLE_Msk; // Enable SysTick with interrupt
}

void SysTick_Disable(void) {
	delay_mode = 0;
	SysTick->CTRL = 0; // Disable SysTick
}

void delay_ms(uint32_t ms) {
	msTicks = ms;
	SysTick_Enable(((System_CoreClock / 1000)), 1); // 1ms tick // 16,000,000 / 1000 = 16,000 ticks
	while (msTicks != 0)
		;
	SysTick_Disable();
}

//void delay_us(uint32_t us) {
// usTicks = us;
// SysTick_Enable(SystemCoreClock / 1000000, 2); // 1µs tick
// while (usTicks != 0)
// ;
// SysTick_Disable();
//}

void delay_us(uint32_t us) {
	uint32_t ticks = us * (System_CoreClock / 1000000); // 1us
	SysTick->LOAD = ticks - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
		//wait for the count down
		;
	SysTick->CTRL = 0; //disable systick
}

void gpio_int_led() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER12_0;    // Output
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_12);   // Push-pull
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12; // High speed
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR12);   // No pull-up/pull-down
	GPIOD->BSRR |= 1 << 12;

}

void SysTick_Handler(void) {
	if (delay_mode == 1 && msTicks > 0) {
		msTicks--;
	}
	if (delay_mode == 2 && usTicks > 0) {
		usTicks--;
	}

	if ((delay_mode == 1) && (msTicks == 0)) {
		SysTick_Disable();
	}

	if ((delay_mode == 2) && (usTicks == 0)) {
		SysTick_Disable();
	}

}

























// /*
//  * SYSTICK.c
//  * Draft1:
//  *  Created on: Aug 2, 2024
//  *      Author: muskaan jain
//  */
// #include "systick.h"
// #include "core_cm4.h" // CMSIS header for ARM Cortex-M4

// // SysTick global counter for tracking elapsed time
// volatile uint32_t sysTickCounter = 0;

// /**
//  * @brief  Initializes the SysTick timer with a specified reload value.
//  * @param  ticks: Reload value for the SysTick timer. The timer will overflow
//  *                 and generate an interrupt when it counts down to zero.
//  * @retval None
//  * @note   If SysTick_Config fails (e.g., if ticks is too large), the function
//  *         enters an infinite loop to indicate an error.
//  */
// void SysTick_Init(uint32_t ticks) {
//     // Configure the SysTick timer with the specified reload value
//     if (SysTick_Config(ticks)) {
//         // Handle error if SysTick_Config fails by entering an infinite loop
//         while (1);
//     }

//     // Set the SysTick interrupt priority to the lowest possible value
//     NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
// }

// /**
//  * @brief  SysTick interrupt service routine (ISR).
//  * @retval None
//  * @note   This function is called by the Cortex-M4 core whenever the SysTick
//  *         timer generates an interrupt. It increments the global sysTickCounter.
//  */
// void SysTick_Handler(void) {
//     // Increment the global tick counter each time SysTick generates an interrupt
//     sysTickCounter++;
// }

// /**
//  * @brief  Gets the current value of the SysTick counter.
//  * @retval Current value of the sysTickCounter.
//  */
// uint32_t SysTick_GetTicks(void) {
//     // Return the current tick count
//     return sysTickCounter;
// }

// /**
//  * @brief  Busy-wait delay function based on the SysTick timer.
//  * @param  delay: Number of ticks to wait. The function will wait until the
//  *                 SysTick counter increments by the specified delay value.
//  * @retval None
//  * @note   This function uses a busy-wait loop to create a delay. It is blocking
//  *         and may not be suitable for time-critical applications.
//  */
// void SysTick_Delay(uint32_t delay) {
//     // Record the start time using the current value of the SysTick counter
//     uint32_t startTick = SysTick_GetTicks();

//     // Wait until the specified delay has elapsed
//     while ((SysTick_GetTicks() - startTick) < delay);
// }
