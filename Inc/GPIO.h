/*
 * GPIO.h
 *
 *  Created on: Aug 19, 2024
 *      Author: muska
 */

#ifndef GPIO_H_
#define GPIO_H_


#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>



//extern volatile uint32_t *reg;  // Global variable definition


/* Enumeration for GPIO modes */
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_GENERAL_PURPOSE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
} GPIO_Mode_t;

/* Enumeration for GPIO output types */
typedef enum {
    GPIO_OUTPUT_TYPE_PUSH_PULL = 0,
    GPIO_OUTPUT_TYPE_OPEN_DRAIN,
	 GPIO_OUTPUT_TYPE_NONE
} GPIO_OutputType_t;

/* Enumeration for GPIO speeds */
typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH,
	GPIO_SPEED_NONE
} GPIO_Speed_t;

/* Enumeration for GPIO pull-up/pull-down configurations */
typedef enum {
    GPIO_PULL_NO = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
	GPIO_NONE
} GPIO_Pull_t;

/* Enumeration for GPIO interrupt edges */
typedef enum {
    GPIO_INTERRUPT_EDGE_RISING = 0,
    GPIO_INTERRUPT_EDGE_FALLING,
    GPIO_INTERRUPT_EDGE_RISING_FALLING
} GPIO_InterruptEdge_t;

/* Enumeration for alternate functions */
typedef enum {
    GPIO_AF_NONE = 0,
    GPIO_AF_ANALOG =0,
    GPIO_AF_SYS = 0,
    GPIO_AF_MCO1=0,
    GPIO_AF_MCO2=0,
    GPIO_AF_RTC_REFIN = 1,
    GPIO_AF_TIM1 =1,
    GPIO_AF_TIM2 = 1,
    GPIO_AF_TIM3 = 2,
    GPIO_AF_TIM4 = 2,
    GPIO_AF_TIM5 = 2,
    GPIO_AF_TIM8 = 3,
    GPIO_AF_TIM9 = 3,
    GPIO_AF_TIM10 = 3,
    GPIO_AF_TIM11 = 3,
    GPIO_AF_I2C1 = 4,
    GPIO_AF_I2C2 = 4,
    GPIO_AF_I2C3 = 4,
    GPIO_AF_SPI1 = 5,
    GPIO_AF_SPI2 = 5,
    GPIO_AF_I2S2 = 5,
    GPIO_AF_I2S2EXT = 5,
    GPIO_AF_SPI3 = 6,
    GPIO_AF_I2S_EXT = 6,
    GPIO_AF_I2S3 = 6,
    GPIO_AF_USART1 = 7,
    GPIO_AF_USART2 = 7,
    GPIO_AF_USART3 = 7,
    GPIO_AF_I2S3EXT = 7,
    GPIO_AF_USART4 = 8,
    GPIO_AF_UART5 = 8,
    GPIO_AF_USART6 = 8,
    GPIO_AF_CAN1 = 9,
    GPIO_AF_CAN2 = 9,
    GPIO_AF_TIM12 = 9,
    GPIO_AF_TIM13 =9,
    GPIO_AF_TIM14 = 9,
    GPIO_AF_OTG_FS1 = 10,
    GPIO_AF_OTG_HS1 = 10,
    GPIO_AF_ETH1 = 11,
    GPIO_AF_FSMC1 = 12,
    GPIO_AF_SDIO1 = 12,
    GPIO_AF_OTG_FS2 = 12,
    GPIO_AF_DCMI1 =13,
    GPIO_AF_EVENTOUT = 15
} GPIO_AlternateFunction_t;

/* GPIO Pin Configuration Structure */
typedef struct GPIO_Configuration{
    uint8_t GPIO_PinNumber;          // GPIO pin number
    GPIO_Mode_t GPIO_PinMode;        // GPIO mode
    GPIO_OutputType_t GPIO_PinOType; // GPIO output type
    GPIO_Speed_t GPIO_PinSpeed;      // GPIO speed
    GPIO_Pull_t GPIO_PinPuPdControl; // GPIO pull-up/pull-down
    GPIO_AlternateFunction_t GPIO_PinAltFunMode; // GPIO alternate function mode
} GPIO_PinConfig_t;

/* GPIO Handle Structure */
typedef struct {
    GPIO_TypeDef *pGPIOx;            // Pointer to GPIO port base address
    GPIO_PinConfig_t GPIO_PinConfig; // GPIO pin configuration settings
} GPIO_Handle_t;

/* API Functions Supported by GPIO Driver */
void GPIO_ClockControl(GPIO_TypeDef* GPIOx);
//void GPIO_Pin_Init(GPIO_Handle_t *handle);
void GPIO_Pin_Init(GPIO_TypeDef *Port, uint8_t pin, GPIO_Mode_t mode, GPIO_OutputType_t output_type, GPIO_Speed_t speed, GPIO_Pull_t pull, GPIO_AlternateFunction_t alternate_function);
bool GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Val);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Val);
//void GPIO_Interrupt_Setup(uint8_t pin, GPIO_InterruptEdge_t edge_select, uint32_t priority);
void GPIO_Pin_Reset(GPIO_TypeDef *Port, uint8_t pin);




#endif /* GPIO_H_ */
