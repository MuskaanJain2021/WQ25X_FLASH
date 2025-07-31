/*
 * GPIO.c
 *
 *  Created on: Aug 19, 2024
 *      Author: muskaan jain
 */


#include "GPIO.h"
/**
 * @brief Reset the GPIO pin to default state.
 * @param Port GPIO port (e.g., GPIOA, GPIOB)
 * @param pin GPIO pin number (0-15)
 */
void GPIO_Pin_Reset(GPIO_TypeDef *Port, uint8_t pin)
{
    // Ensure valid pin number
    if (pin > 15) {
        return; // Invalid pin number
    }

    // Reset mode to input
    Port->MODER &= ~(0x03 << (pin * 2)); // Clear mode bits

    // Reset output type to push-pull
    Port->OTYPER &= ~(1 << pin); // Clear output type bit

    // Reset speed to low
    Port->OSPEEDR &= ~(0x03 << (pin * 2)); // Clear speed bits

    // Reset pull-up/pull-down to no pull
    Port->PUPDR &= ~(0x03 << (pin * 2)); // Clear pull-up/pull-down bits

    // Reset alternate function to AF0 (default)
    if (pin < 8) {
        Port->AFR[0] &= ~(0x0F << (pin * 4)); // Clear alternate function bits
    } else {
        Port->AFR[1] &= ~(0x0F << ((pin - 8) * 4)); // Clear alternate function bits
    }
}



void GPIO_ClockControl(GPIO_TypeDef* GPIOx)
{
    uint32_t bit = 0;

    // Enable GPIO clock based on port
    if (GPIOx == GPIOA)
    {
        bit = RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOx == GPIOB)
    {
        bit = RCC_AHB1ENR_GPIOBEN;
    }
    else if (GPIOx == GPIOC)
    {
        bit = RCC_AHB1ENR_GPIOCEN;
    }
    else if (GPIOx == GPIOD)
    {
        bit = RCC_AHB1ENR_GPIODEN;
    }
    else if (GPIOx == GPIOE)
    {
        bit = RCC_AHB1ENR_GPIOEEN;
    }
    else if (GPIOx == GPIOH)
    {
        bit = RCC_AHB1ENR_GPIOHEN;
    }
    else
    {
        // Handle error case
        return;
    }

    RCC->AHB1ENR |= bit;
}

void GPIO_Pin_Init(GPIO_TypeDef *Port, uint8_t pin, GPIO_Mode_t mode, GPIO_OutputType_t output_type, GPIO_Speed_t speed, GPIO_Pull_t pull, GPIO_AlternateFunction_t alternate_function)

{
    // Ensure valid pin number
    if (pin > 15)
    {
        return; // Invalid pin number
    }

    // Enable GPIO clock
    GPIO_ClockControl(Port);

    // Configure mode
    Port->MODER &= ~(0x03 << (pin * 2)); // Clear mode bits
    Port->MODER |= (mode << (pin * 2)); // Set new mode


    // Configure output type
    if (output_type == GPIO_OUTPUT_TYPE_OPEN_DRAIN)
    {
        Port->OTYPER |= (1 << pin);
    }
    else
    {
        Port->OTYPER &= ~(1 << pin);
    }

    // Configure speed
    Port->OSPEEDR &= ~(0x03 << (pin * 2)); // Clear speed bits
    Port->OSPEEDR |= (speed << (pin * 2)); // Set new speed

    // Configure pull-up/pull-down
    Port->PUPDR &= ~(0x03 << (pin * 2)); // Clear pull-up/pull-down bits
    Port->PUPDR |= (pull << (pin * 2)); // Set new pull-up/pull-down

    // Configure alternate function
    if (pin < 8)
    {
        Port->AFR[0] &= ~(0x0F << (pin * 4)); // Clear alternate function bits
        Port->AFR[0] |= (alternate_function << (pin * 4)); // Set new alternate function
    }
    else
    {
        Port->AFR[1] &= ~(0x0F << ((pin - 8) * 4)); // Clear alternate function bits
        Port->AFR[1] |= (alternate_function << ((pin - 8) * 4)); // Set new alternate function
    }
}
//void GPIO_Pin_Init(GPIO_Handle_t *handle)
//{
//    // Validate the handle and pin number
//    if (!handle || handle->GPIO_PinConfig.GPIO_PinNumber > 15)
//    {
//        return; // Invalid handle or pin number
//    }
//
//    // Local variable to improve readability
//    GPIO_TypeDef *Port = handle->pGPIOx;
//    uint8_t pin = handle->GPIO_PinConfig.GPIO_PinNumber;
//    GPIO_Mode_t mode = handle->GPIO_PinConfig.GPIO_PinMode;
//    GPIO_OutputType_t output_type = handle->GPIO_PinConfig.GPIO_PinOType;
//    GPIO_Speed_t speed = handle->GPIO_PinConfig.GPIO_PinSpeed;
//    GPIO_Pull_t pull = handle->GPIO_PinConfig.GPIO_PinPuPdControl;
//    GPIO_AlternateFunction_t alternate_function = handle->GPIO_PinConfig.GPIO_PinAltFunMode;
//
//    // Enable GPIO clock
//    GPIO_ClockControl(Port);
//
//    // Configure mode
//    Port->MODER &= ~(0x03 << (pin * 2)); // Clear mode bits
//    Port->MODER |= (mode << (pin * 2));  // Set new mode
//
//    // Configure output type
//    Port->OTYPER &= ~(1 << pin);         // Clear current output type
//    if (output_type == GPIO_OUTPUT_TYPE_OPEN_DRAIN)
//    {
//        Port->OTYPER |= (1 << pin);      // Set output type to Open Drain
//    }
//
//    // Configure speed
//    Port->OSPEEDR &= ~(0x03 << (pin * 2)); // Clear speed bits
//    Port->OSPEEDR |= (speed << (pin * 2)); // Set new speed
//
//    // Configure pull-up/pull-down
//    Port->PUPDR &= ~(0x03 << (pin * 2));   // Clear pull-up/pull-down bits
//    Port->PUPDR |= (pull << (pin * 2));    // Set new pull-up/pull-down
//
//    // Configure alternate function if required
//    if (mode == GPIO_MODE_ALTERNATE_FUNCTION) {
//        if (pin < 8) {
//            Port->AFR[0] &= ~(0x0F << (pin * 4)); // Clear alternate function bits for lower half
//            Port->AFR[0] |= (alternate_function << (pin * 4)); // Set new alternate function
//        } else {
//            Port->AFR[1] &= ~(0x0F << ((pin - 8) * 4)); // Clear alternate function bits for upper half
//            Port->AFR[1] |= (alternate_function << ((pin - 8) * 4)); // Set new alternate function
//        }
//    }
//}

bool GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    // Return true if the pin is high, false if it is low
    return (bool)((pGPIOx->IDR >> PinNumber) & 0x01);
}

uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
    // Return the value of the input data register
    return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Val)
{
    if (Val == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit position corresponding to the PinNumber
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Write 0 to the output data register at the bit position corresponding to the PinNumber
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    // Toggle the output data register at the bit position corresponding to the PinNumber
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Val)
{
    // Write the value to the output data register
    pGPIOx->ODR = Val;
}

/**
 * @brief  Sets up an interrupt for a specific GPIO pin.
 * @param  pin: GPIO pin number (0-15).
 * @param  edge_select: 0 for rising edge, 1 for falling edge, 2 for both edges.
 * @param  priority: Priority level for the NVIC interrupt.
// */
//void GPIO_Interrupt_Setup(int pin, int edge_select, uint32_t priority)
//{
//    // Enable interrupt mask register for the pin
//    EXTI->IMR |= (1 << pin);
//
//    // Configure edge selection
//    switch (edge_select) {
//        case 0:
//            EXTI->RTSR |= (1 << pin); // Rising edge
//            break;
//        case 1:
//            EXTI->FTSR |= (1 << pin); // Falling edge
//            break;
//        case 2:
//            EXTI->RTSR |= (1 << pin); // Rising edge
//            EXTI->FTSR |= (1 << pin); // Falling edge
//            break;
//        default:
//            return; // Invalid edge select option
//    }
//
//    // Configure NVIC based on pin number
//    IRQn_Type irq;
//    if (pin <= 4) {
//        irq = (IRQn_Type)(EXTI0_IRQn + pin);
//    } else if (pin <= 9) {
//        irq = EXTI9_5_IRQn;
//    } else if (pin <= 15) {
//        irq = EXTI15_10_IRQn;
//    } else {
//        return; // Invalid pin number
//    }
//    NVIC_SetPriority(irq, priority);
//    NVIC_EnableIRQ(irq);
//}
