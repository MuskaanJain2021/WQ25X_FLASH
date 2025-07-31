/*
 * EXTI.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Muskaan Jain
 */

#ifndef EXTI_H
#define EXTI_H

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "GPIO.h"

/* EXTI Trigger Types */
typedef enum {
    EXTI_TRIGGER_RISING,
    EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_BOTH
} EXTI_Trigger_t;


/* Function Pointer Type for EXTI ISR Handlers */
typedef void (*EXTI_ISR_Handler)(void);
void NVIC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void NVIC_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
bool EXTI_Config(GPIO_TypeDef *port, uint8_t pin, EXTI_Trigger_t edge, uint32_t priority,EXTI_ISR_Handler handler);
void EXTI_ClearPending(uint8_t pin);
void EXTI_Dispatcher(void);

#endif /* EXTI__H_ */
