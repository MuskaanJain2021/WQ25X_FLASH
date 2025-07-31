#include "EXTI.h"
#include <string.h>
//#include "NVIC_Config.h"
#include <stdlib.h>
static EXTI_ISR_Handler EXTI_ISRs[16] = {0};
void EXTI_ClearPending(uint8_t pin) {
	if (pin > 15)
		return; // Invalid pin
	EXTI->PR |= (1 << pin); // Writing 1 clears the pending bit
}


//void EXTI_Dispatcher(void) {
//    uint32_t pending = EXTI->PR;  // Read once to avoid multiple accesses
//    for (uint8_t pin = 0; pin < 16; pin++) {
//        if (pending & (1 << pin)) {
//            if (EXTI_ISRs[pin]) {
//                EXTI_ISRs[pin]();
//            }
//            EXTI_ClearPending(pin);  // Clear the pending bit
//        }
//    }
//}

static uint8_t GPIO_PORT_TO_EXTI_PORT_CODE(GPIO_TypeDef *port) {
	if (port == GPIOA)
		return 0;
	if (port == GPIOB)
		return 1;
	if (port == GPIOC)
		return 2;
	if (port == GPIOD)
		return 3;
	if (port == GPIOE)
		return 4;
	if (port == GPIOF)
		return 5;
	if (port == GPIOG)
		return 6;
	if (port == GPIOH)
		return 7;
	return 0xFF; // Invalid port
}

void NVIC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	 //IRQNumber=abs(IRQNumber);
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
				{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	} else {
		if (IRQNumber <= 31) {
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 6 && IRQNumber < 96) {
			//program ICER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}

}

void NVIC_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	uint32_t absIRQNumber = abs(IRQNumber);

	uint8_t iprx = absIRQNumber / 4;
	uint8_t iprx_section = absIRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

bool EXTI_Config(GPIO_TypeDef *port, uint8_t pin, EXTI_Trigger_t edge,uint32_t priority, EXTI_ISR_Handler handler) {
	if ( !port || pin > 15 ||!handler) {
		return false; // Invalid parameters
	}

	/* Enable SYSCFG Clock */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* Configure SYSCFG_EXTICR for the given pin */
	uint8_t port_code = GPIO_PORT_TO_EXTI_PORT_CODE(port);
	if (port_code == 0xFF) {
		return false; // Invalid port
	}

	/* Unmask the interrupt */
	EXTI->IMR |= (1 << pin);

	// Each SYSCFG_EXTICR register handles 4 EXTI lines
	uint8_t exticr_index = pin / 4;
	uint8_t exticr_position = (pin % 4) * 4;

	SYSCFG->EXTICR[exticr_index] &= ~(0xF << exticr_position); // Clear existing
	SYSCFG->EXTICR[exticr_index] |= (port_code << exticr_position); // Set new

	/* Configure EXTI Trigger */
	if (edge == EXTI_TRIGGER_RISING || edge == EXTI_TRIGGER_BOTH) {
		EXTI->RTSR |= (1 << pin); // Enable Rising trigger
	} else {
		EXTI->RTSR &= ~(1 << pin); // Disable Rising trigger
	}

	if (edge == EXTI_TRIGGER_FALLING || edge == EXTI_TRIGGER_BOTH) {
		EXTI->FTSR |= (1 << pin); // Enable Falling trigger
	} else {
		EXTI->FTSR &= ~(1 << pin); // Disable Falling trigger
	}

	/* Assign the ISR handler */
	EXTI_ISRs[pin] = handler;
	/* Determine IRQ Number based on pin */
	IRQn_Type irqNumber = (pin < 5) ? (IRQn_Type)(EXTI0_IRQn + pin) : (pin < 10) ? EXTI9_5_IRQn : EXTI15_10_IRQn;


	/* Set NVIC Priority and Enable IRQ */
	NVIC_IRQPriorityConfig(irqNumber, priority);
	NVIC_IRQInterruptConfig(irqNumber, ENABLE);

	return true;
}

