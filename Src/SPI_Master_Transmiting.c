/*
 * SPI_Master_Transmiting.c
 *
 *  Created on: Jan 14, 2025
 *      Author: muska
 */

#include <GPIO.h>
#include <main.h>
#include "SPI.h"
#include <string.h>
#include <stm32f407xx.h>

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB9 --> SPI2_NSS
 * ALT function mode : 5
 */

// SPI Handle for SPI2 (can be used for other SPI peripherals as well)
SPI_Handle_t spi_handle;
GPIO_Handle_t gpio_handle;

// Function to configure SPI GPIO pins
//void SPI_GPIO_Init(SPI_Handle_t* spiHandle, GPIO_Handle_t* gpioHandle) {
// SPI_GPIO_Init(spiHandle, gpioHandle);
//}

// Function to configure SPI settings
void SPI_Init_Config(SPI_Handle_t *spiHandle, SPI_TypeDef *SPIx,
		GPIO_TypeDef *GPIOx, uint32_t clkPin, uint32_t mosiPin,
		uint32_t misoPin) {
	// Set SPI configuration
	spiHandle->pSPIx = SPIx;
	spiHandle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spiHandle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_MASTER_FD; // Full-Duplex
	spiHandle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spiHandle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;  // 8-bit data frame format
	spiHandle->SPIConfig.SPI_CPOL = SPI_CPOL_HIGH; // idle state is high
	spiHandle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;  // data captured on 1st edge
	spiHandle->SPIConfig.SPI_SSM = SPI_SSM_EN;  // Hardware NSS Management
	spiHandle->SPIConfig.SPI_SSOE = SPI_SSOE_DI; // if set Automatically drive NSS low during tx of data
	spiHandle->SPIConfig.SPI_SSI = SPI_SSI_EN;// ENABLE internally NSS connected to GND

	spiHandle->SPIConfig.GPIO_NSS.pGPIOx = GPIOB;
	spiHandle->SPIConfig.GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	spiHandle->SPIConfig.GPIO_CLK.pGPIOx = GPIOB;
	spiHandle->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber = 13;
	spiHandle->SPIConfig.GPIO_MOSI.pGPIOx = GPIOB;
	spiHandle->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber = 15;

	spiHandle->SPIConfig.GPIO_MISO.pGPIOx = GPIOB;
	spiHandle->SPIConfig.GPIO_MISO.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_ClockControl(GPIOB);

	// SPI Pin Configuration

}

// SPI Initialization function
void SPI_Init_Peripheral(SPI_Handle_t *spiHandle) {
	SPI_Init(spiHandle);
}

// SPI Enable function
void SPI_Enable_Peripheral(SPI_Handle_t *spiHandle) {
	SPI_Enable(spiHandle);
}

// SPI Reset Configuration function
int8_t SPI_Reset_Configuration(SPI_Handle_t *spiHandle) {
	return SPI_Configuration_Reset(spiHandle);
}

// Main function
int main(void) {
	char user_data[] = "HELLO DATA SPI";
	// Initialize the system clock
	//MCU_Clock_Setup();

	// Set up SPI2 configuration (This can be reused for other SPI peripherals)
	SPI_Init_Config(&spi_handle, SPI2, GPIOB, GPIO_PIN_13, GPIO_PIN_15,
	GPIO_PIN_14);

	// Initialize SPI2 peripheral
	SPI_Init_Peripheral(&spi_handle);

	// Reset SPI configuration to default values and check result
	//int8_t result = SPI_Reset_Configuration(&spi_handle);

	// Send data via SPI in polling mode
	SPI_SEND_DATA_POLLING(&spi_handle, (uint8_t*) user_data, strlen(user_data));
	// Handle error (invalid SPI peripheral),De-initialize SPI peripheral (if needed after use as u don't want to use spix where x=1,2,3 for further communication )
	SPI_DeInit(&spi_handle);

	// Main loop
	while (1) {

	}

}

