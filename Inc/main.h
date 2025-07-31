#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f407xx.h"
//#include "SYSTICK.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Function prototypes
uint32_t SysTick_Config(uint32_t ticks);

// Correct placement of extern variable declaration
extern uint32_t APB1CLK_SPEED;
extern uint32_t APB2CLK_SPEED;

typedef struct {
    bool Enable;
    uint32_t Interrup_Flags;
} Interrupts;

#define __weak   __attribute__((weak))

#define SPI_Debug_Flag 0

__STATIC_INLINE int32_t SystemAPB1_Clock_Speed(void) {
    return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}

__STATIC_INLINE int32_t SystemAPB2_Clock_Speed(void) {
    return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
}

__STATIC_INLINE void MCU_Clock_Setup(void) {
    uint8_t pll_m = 8;
    uint16_t pll_n = 336;
    uint8_t pll_p = 0;
    uint8_t pll_q = 7;

    RCC->PLLCFGR = 0x00000000;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR |= (pll_q << 24) | (pll_p << 16) | (pll_n << 6) | (pll_m << 0);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 168);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

__STATIC_INLINE int I2S_Clock_Init(void) {
    uint32_t RCC_PLLI2SCFGR = 0;
    uint32_t plli2s_n = 384;
    uint32_t plli2s_r = 5;
    RCC_PLLI2SCFGR = plli2s_n << 6;
    RCC_PLLI2SCFGR |= plli2s_r << 28;
    RCC->PLLI2SCFGR = RCC_PLLI2SCFGR;
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY)) {}
    return 0;
}

__STATIC_INLINE uint32_t Delay_Config(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    return 0;
}

__STATIC_INLINE void Delay_DeInit(void) {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

__STATIC_INLINE uint32_t Delay_us(float us) {
    SysTick->LOAD = 167 * us;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    return 0;
}

__STATIC_INLINE uint32_t Delay_ms(float ms) {
    unsigned long x = 0x29040 * ms;
    SysTick->LOAD = x;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    return 0;
}

__STATIC_INLINE uint32_t Delay_s(unsigned long s) {
    s = s * 1000;
    for (; s > 0; s--) {
        Delay_ms(1);
    }
    return 0;
}

__STATIC_INLINE float Time_Stamp_Start(void) {
    float temp = 0;
    SysTick->CTRL = 0;
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (SysTick->VAL != 0);
    temp = (float)(SysTick->VAL / (SystemCoreClock));
    return temp;
}

__STATIC_INLINE float Time_Stamp_End(void) {
    float temp = 0;
    temp = (float)(SysTick->VAL / (SystemCoreClock));
    return temp;
}

__STATIC_INLINE void separateFractionAndIntegral(double number, double *fractionalPart, double *integralPart) {
    *integralPart = (double)((int64_t)number);
    *fractionalPart = number - *integralPart;
}

#endif /* MAIN_H_ */
