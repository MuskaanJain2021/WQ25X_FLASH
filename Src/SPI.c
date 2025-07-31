/*
 * SPI.c
 *
 *  Created on: Jan 6, 2025
 *      Author: muska
 */

#include <SPI.h>
#include <GPIO.h>

// Define the SPI pins for SPI1, SPI2, and SPI3
SPI_Pin spiPins = { ._SPI1 = { .CLK1 = { .PA5 = GPIO_PIN_5, .PB3 = GPIO_PIN_3 }, // SPI1 Clock (PA5, PB3)
		.MOSI1 = { .PA7 = GPIO_PIN_7, .PB5 = GPIO_PIN_5 }, // SPI1 MOSI (PA7, PB5)
		.MISO1 = { .PA6 = GPIO_PIN_6, .PB4 = GPIO_PIN_4 } // SPI1 MISO (PA6, PB4)
}, ._SPI2 = { .CLK2 = { .PB10 = GPIO_PIN_10, .PB13 = GPIO_PIN_13 }, // SPI2 Clock (PB10, PB13)
		.MOSI2 = { .PC3 = GPIO_PIN_3, .PB15 = GPIO_PIN_15 }, // SPI2 MOSI (PC3, PB15)
		.MISO2 = { .PC2 = GPIO_PIN_2, .PB14 = GPIO_PIN_14 } // SPI2 MISO (PC2, PB14)
}, ._SPI3 = { .CLK3 = { .PC10 = GPIO_PIN_10, .PB3 = GPIO_PIN_3 }, // SPI3 Clock (PC10, PB3)
		.MOSI3 = { .PC12 = GPIO_PIN_12, .PB5 = GPIO_PIN_5 }, // SPI3 MOSI (PC12, PB5)
		.MISO3 = { .PC11 = GPIO_PIN_11, .PB4 = GPIO_PIN_4 } // SPI3 MISO (PC11, PB4)
} };

void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		if (pSPIx == SPI2)
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		if (pSPIx == SPI3)
			RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	} else {
		if (pSPIx == SPI1)
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
		if (pSPIx == SPI2)
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
		if (pSPIx == SPI3)
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
	}
}
void SPI_Enable(SPI_Handle_t *config) {
	config->pSPIx->CR1 |= SPI_CR1_SPE;
}

void SPI_Disable(SPI_Handle_t *config) {
	config->pSPIx->CR1 &= ~SPI_CR1_SPE;
}

void SPI_DeInit(SPI_Handle_t *config) {
	if (config->pSPIx == SPI1)
		RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	if (config->pSPIx == SPI2)
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
	if (config->pSPIx == SPI3)
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST;
}

// Helper function to initialize GPIO pins for SPI
void SPI_GPIO_Pin_Init(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF,
		uint8_t Speed) {
	GPIO_Pin_Init(GPIOx, Pin, GPIO_MODE_ALTERNATE_FUNCTION,
			GPIO_OUTPUT_TYPE_PUSH_PULL, Speed, GPIO_PULL_NO, AF);
}

int8_t SPI_Configuration_Reset(SPI_Handle_t *config) {
	uint8_t retval = 0;
	if (config->pSPIx == SPI1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	else if (config->pSPIx == SPI2)
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
	else if (config->pSPIx == SPI3)
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
	else {
		retval = -1; // Invalid SPI peripheral
		return retval; // Exit early if SPI peripheral is invalid
	}

	config->SPIConfig.GPIO_NSS.pGPIOx = GPIOB;
	config->SPIConfig.GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;

	config->SPIConfig.GPIO_CLK.pGPIOx = GPIOB;
	config->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;

	config->SPIConfig.GPIO_MOSI.pGPIOx = GPIOB;
	config->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;

	config->SPIConfig.GPIO_MISO.pGPIOx = GPIOB;
	config->SPIConfig.GPIO_MISO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;

	config->SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	config->SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
	config->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	config->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	config->SPIConfig.SPI_FrameFormat = SPI_FRAME_FORMAT_MSB_FIRST;
	config->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_MASTER_FD;
	config->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	return retval;
}
uint8_t SPI_GetFlags_Status(SPI_TypeDef *SPIx, uint32_t FlagName) {
	return (SPIx->SR & FlagName) ? FLAG_SET : FLAG_RESET;
}
void SPI_SSOE_Config(SPI_TypeDef *pSPIx, uint8_t SSOE_Status) {
	if (SSOE_Status == ENABLE) {
		pSPIx->CR2 |= SPI_CR2_SSOE;  // Enable SSOE (Slave Select Output Enable)
	} else {
		pSPIx->CR2 &= ~SPI_CR2_SSOE;  // Disable SSOE
	}
}

void SPI_SSI_Config(SPI_TypeDef *pSPIx, uint8_t SSI_Status) {
	if (SSI_Status == ENABLE) {
		pSPIx->CR1 |= SPI_CR1_SSI;  // Set SSI bit (NSS high)
	} else {
		pSPIx->CR1 &= ~SPI_CR1_SSI;  // Clear SSI bit (NSS low)
	}
}
/**
 * @brief  Initialize SPI-related GPIO pins based on bus configuration and custom pin map.
 * @param  config1 : Pointer to SPI handle structure with desired SPI bus config.
 * @retval None
 *   This function checks which SPI peripheral is used,
     then configures CLK, MISO, MOSI pins with the correct
     alternate function and mode (full-duplex, half-duplex, etc.).
 */

void SPI_GPIO_Init1(SPI_Handle_t *config1) {
	if (config1->pSPIx == SPI1) {
		if ((config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_MASTER_FD)
				|| (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SLAVE_FD)) {

			// CLK Pin Initialization using SPI_GPIO_Pin_Init
			if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB10) {
				SPI_GPIO_Pin_Init(GPIOB, GPIO_PIN_10, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			}
		}

		else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.CLK1.PB3) {
			SPI_GPIO_Pin_Init(GPIOB, GPIO_PIN_3, GPIO_AF_SPI1,
					GPIO_SPEED_VERY_HIGH);
		}

		if (config1->SPIConfig.GPIO_MISO.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_MISO.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MISO1.PA6) {
			SPI_GPIO_Pin_Init(GPIOA, GPIO_PIN_6, GPIO_AF_SPI1,
					GPIO_SPEED_VERY_HIGH);
		} else if (config1->SPIConfig.GPIO_MISO.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_MISO.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MISO1.PB4)
			SPI_GPIO_Pin_Init(GPIOB, 4, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PA7)
			SPI_GPIO_Pin_Init(GPIOA, 7, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);
		else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PB5)
			SPI_GPIO_Pin_Init(GPIOB, 5, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		config1->pSPIx->CR1 &= ~SPI_CR1_BIDIMODE; //CLEAR BIDIMODE
		//config->pSPIx->CR1 &= ~SPI_CR1_RXONLY; // CLEAR RXONLY -Enable RX and TX
	}

	else if (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_MASTER_HD
			|| config1->SPIConfig.SPI_BusConfig
					== SPI_BUS_CONFIG_SIMPLEX_SLAVE_RXONLY
			|| config1->SPIConfig.SPI_BusConfig
					== SPI_BUS_CONFIG_SIMPLEX_MASTER_TXONLY) {
		if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.CLK1.PA5)
			SPI_GPIO_Pin_Init(GPIOA, 5, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);
		else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.CLK1.PB3)
			SPI_GPIO_Pin_Init(GPIOB, 3, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PA7)
			SPI_GPIO_Pin_Init(GPIOA, 7, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);
		else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PB5)
			SPI_GPIO_Pin_Init(GPIOB, 5, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		config1->pSPIx->CR1 |= SPI_CR1_BIDIMODE;
		config1->pSPIx->CR1 |= SPI_CR1_BIDIOE;
	} else if (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SLAVE_HD
			|| config1->SPIConfig.SPI_BusConfig
					== SPI_BUS_CONFIG_SIMPLEX_MASTER_RXONLY
			|| config1->SPIConfig.SPI_BusConfig
					== SPI_BUS_CONFIG_SIMPLEX_SLAVE_TXONLY) {
		if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.CLK1.PA5)
			SPI_GPIO_Pin_Init(GPIOA, 5, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);
		else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.CLK1.PB3)
			SPI_GPIO_Pin_Init(GPIOB, 3, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOA
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PA7)
			SPI_GPIO_Pin_Init(GPIOA, 7, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);
		else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
				&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
						== spiPins._SPI1.MOSI1.PB5)
			SPI_GPIO_Pin_Init(GPIOB, 5, GPIO_AF_SPI1, GPIO_SPEED_VERY_HIGH);

		config1->pSPIx->CR1 |= SPI_CR1_BIDIMODE;
		config1->pSPIx->CR1 &= ~SPI_CR1_BIDIOE;
	}

	if (config1->pSPIx == SPI2) {
		if ((config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_MASTER_FD)
				|| (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SLAVE_FD)) {
			if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB10)
				SPI_GPIO_Pin_Init(GPIOB, 10, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB13)
				SPI_GPIO_Pin_Init(GPIOB, 13, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);

			if (config1->SPIConfig.GPIO_MISO.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_MISO.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MISO2.PB14)

				SPI_GPIO_Pin_Init(GPIOB, 14, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOC
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MISO2.PC2)

				SPI_GPIO_Pin_Init(GPIOC, 2, GPIO_AF_SPI2, GPIO_SPEED_VERY_HIGH);

			if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MOSI2.PB15)
				SPI_GPIO_Pin_Init(GPIOB, 15, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOC
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MOSI2.PC3)
				SPI_GPIO_Pin_Init(GPIOC, 3, GPIO_AF_SPI2, GPIO_SPEED_VERY_HIGH);

			config1->pSPIx->CR1 &= ~SPI_CR1_BIDIMODE; //CLEAR BIDIMODE
			//config->pSPIx->CR1 &= ~SPI_CR1_RXONLY; // CLEAR RXONLY -Enable RX and TX
		}

		else if (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_MASTER_HD
				|| config1->SPIConfig.SPI_BusConfig
						== SPI_BUS_CONFIG_SIMPLEX_SLAVE_RXONLY
				|| config1->SPIConfig.SPI_BusConfig
						== SPI_BUS_CONFIG_SIMPLEX_MASTER_TXONLY) {

			if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB10)
				SPI_GPIO_Pin_Init(GPIOB, 10, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB13)
				SPI_GPIO_Pin_Init(GPIOB, 13, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);

			if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MOSI2.PB15)
				SPI_GPIO_Pin_Init(GPIOB, 15, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
					== spiPins._SPI2.MOSI2.PC3)
				SPI_GPIO_Pin_Init(GPIOC, 3, GPIO_AF_SPI2, GPIO_SPEED_VERY_HIGH);

			config1->pSPIx->CR1 |= SPI_CR1_BIDIMODE;
			config1->pSPIx->CR1 |= SPI_CR1_BIDIOE;
		} else if (config1->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SLAVE_HD
				|| config1->SPIConfig.SPI_BusConfig
						== SPI_BUS_CONFIG_SIMPLEX_MASTER_RXONLY
				|| config1->SPIConfig.SPI_BusConfig
						== SPI_BUS_CONFIG_SIMPLEX_SLAVE_TXONLY) {

			if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB10)
				SPI_GPIO_Pin_Init(GPIOB, 10, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_CLK.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_CLK.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.CLK2.PB13)
				SPI_GPIO_Pin_Init(GPIOB, 13, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);

			if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOB
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MOSI2.PB15)
				SPI_GPIO_Pin_Init(GPIOB, 15, GPIO_AF_SPI2,
						GPIO_SPEED_VERY_HIGH);
			else if (config1->SPIConfig.GPIO_MOSI.pGPIOx == GPIOC
					&& config1->SPIConfig.GPIO_MOSI.GPIO_PinConfig.GPIO_PinNumber
							== spiPins._SPI2.MOSI2.PC3)
				SPI_GPIO_Pin_Init(GPIOC, 3, GPIO_AF_SPI2, GPIO_SPEED_VERY_HIGH);

			config1->pSPIx->CR1 |= SPI_CR1_BIDIMODE;
			config1->pSPIx->CR1 &= ~SPI_CR1_BIDIOE;
		}

	}
}

/**
 * @brief  Initialize the SPI peripheral using parameters from SPI handle.
 * @param  pSPIHandle : Pointer to SPI handle structure.
 * @retval None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

// 1. Disable SPI before configuration
	pSPIHandle->pSPIx->CR1 &= ~SPI_CR1_SPE;

// 2. Configure SPI setting
// Enable the clock for the specified SPI peripheral
	if (pSPIHandle->pSPIx == SPI1) {
		SPI_PeriClockControl(SPI1, ENABLE);
	} else if (pSPIHandle->pSPIx == SPI2) {
		SPI_PeriClockControl(SPI2, ENABLE);
	} else if (pSPIHandle->pSPIx == SPI3) {
		SPI_PeriClockControl(SPI3, ENABLE);
	}

	// Initialize NSS pin based on device mode
	if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
		GPIO_Pin_Init(pSPIHandle->SPIConfig.GPIO_NSS.pGPIOx,
				pSPIHandle->SPIConfig.GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber,
				GPIO_MODE_GENERAL_PURPOSE_OUTPUT, GPIO_SPEED_VERY_HIGH,
				GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_OUTPUT_TYPE_PUSH_PULL,
				GPIO_AF_NONE);
	} else if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE) {
		GPIO_Pin_Init(pSPIHandle->SPIConfig.GPIO_NSS.pGPIOx,
				pSPIHandle->SPIConfig.GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber,
				GPIO_MODE_GENERAL_PURPOSE_OUTPUT, GPIO_SPEED_VERY_HIGH,
				GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_OUTPUT_TYPE_PUSH_PULL,
				GPIO_AF_NONE);

	}

// Initialize the GPIO pins for the SPI peripheral
	SPI_GPIO_Init1(pSPIHandle);

// Configure the SPI settings based on the SPI_Config_t structure
	SPI_TypeDef *pSPIx = pSPIHandle->pSPIx;

// Reset the CR1 register for fresh configuration
	pSPIx->CR1 = 0;

// Set the device mode (master/slave)
	pSPIx->CR1 &= ~SPI_CR1_MSTR;  // Clear master/slave bit
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_Pos;


// Set the clock speed (baud rate prescaler)
	pSPIx->CR1 &= ~SPI_CR1_BR;  // Clear the BR bits
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_Pos;

// Set the data frame format (8-bit or 16-bit)
	pSPIx->CR1 &= ~SPI_CR1_DFF;  // Clear DFF bit
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_Pos;

// Set the frame format (MSB-first or LSB-first)
	pSPIx->CR1 &= ~SPI_CR1_LSBFIRST;  // Clear LSBFirst bit
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_FrameFormat << SPI_CR1_LSBFIRST_Pos;

// Set the clock polarity (CPOL)
	pSPIx->CR1 &= ~SPI_CR1_CPOL;  // Clear CPOL bit
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_Pos;

// Set the clock phase (CPHA)
	pSPIx->CR1 &= ~SPI_CR1_CPHA;  // Clear CPHA bit
	pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_Pos;

// Enable or disable Software Slave Management (SSM)
//pSPIx->CR1 &= ~SPI_CR1_SSM;  // Clear SSM bit
//pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_Pos;
	if (pSPIHandle->SPIConfig.SPI_SSM) {
		// Enable Software Slave Management
		pSPIx->CR1 |= SPI_CR1_SSM;  // SSM = 1 Enable SSM
		SPI_SSI_Config(pSPIx, ENABLE);  // Set SSI to keep NSS high

	} else {
		// Disable Software Slave Management
		pSPIx->CR1 &= ~SPI_CR1_SSM;  // SSM = 0
		SPI_SSI_Config(pSPIx, DISABLE);  // Clear SSI to manage NSS in hardware
		// SSI bit doesn't matter in this case (hardware NSS management)

	}

	// Configure SSOE in CR2 for master mode not needed in slave mode
	if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
		SPI_SSOE_Config(pSPIx, pSPIHandle->SPIConfig.SPI_SSOE); // Enable or disable SSOE as needed
	} else {
		// In slave mode, disable SSOE
		SPI_SSOE_Config(pSPIx, DISABLE);
	}
// Enable the SPI peripheral
	pSPIx->CR1 |= SPI_CR1_SPE;  // Set the SPE bit to enable SPI
}


/**
 * @brief  Update the data frame format (8bit/16bit) on the fly.
 * @param  config : Pointer to SPI handle structure.
 * @retval None
 */
void SPI_Data_Format_Update(SPI_Handle_t *config) {
	SPI_Disable(config);
	if (config->SPIConfig.SPI_DFF == SPI_DFF_8BITS)
		config->pSPIx->CR1 &= ~SPI_CR1_DFF;
	else if (config->SPIConfig.SPI_DFF == SPI_DFF_16BITS)
		config->pSPIx->CR1 |= SPI_CR1_DFF;
	SPI_Enable(config);
}


/**
 * @brief  Send data in polling mode.
 * @param  SPIHandle : Pointer to SPI handle structure.
 * @param  TX_BUFFER : Pointer to the transmit buffer.
 * @param  LENGTH    : Number of bytes to send.
 * @retval None
 */
void SPI_SEND_DATA_POLLING(SPI_Handle_t *SPIHandle, uint8_t *TX_BUFFER,
		uint32_t LENGTH) {

	while (LENGTH > 0) {
		// POLLING: Wait until TXE (Transmit Buffer Empty) flag is set
		while (SPI_GetFlags_Status(SPIHandle->pSPIx, _SPI_SR_TXE) == FLAG_RESET);

		// Check the DFF (Data Frame Format) bit to determine data size
		if (SPIHandle->pSPIx->CR1 & SPI_CR1_DFF) {
			// 16-bit data frame
			SPIHandle->pSPIx->DR = *((uint16_t*) TX_BUFFER); // Cast and dereference as 16-bit
			LENGTH -= 2;                           // Decrease length by 2 bytes
			TX_BUFFER += 2;                    // Move buffer pointer by 2 bytes
		} else {
			// 8-bit data frame
			SPIHandle->pSPIx->DR = *TX_BUFFER; // Write 8-bit data to the data register
			LENGTH--;                               // Decrease length by 1 byte
			TX_BUFFER++;                        // Move buffer pointer by 1 byte
		}
	}

// POLLING: Wait until BSY (Busy) flag is reset to ensure transmission completion
	while (SPI_GetFlags_Status(SPIHandle->pSPIx, _SPI_SR_BSY) == FLAG_SET);
		;
}

/**
 * @brief  Receive data in polling mode.
 * @param  SPIHandle : Pointer to SPI handle structure.
 * @param  RX_BUFFER : Pointer to the receive buffer (16-bit aligned).
 * @param  LENGTH    : Number of bytes to receive.
 * @retval None
 */
void SPI_RECIEVE_DATA_POLLING(SPI_Handle_t *SPIHandle, uint16_t *RX_BUFFER,
		uint32_t LENGTH) {
	while (LENGTH > 0) {
		// POLLING: Wait until RXNE (RECIEVE Buffer NOT Empty) flag is set
		while (SPI_GetFlags_Status(SPIHandle->pSPIx, _SPI_SR_RXNE) == FLAG_RESET)
			;

		// Check the DFF (Data Frame Format) bit to determine data size
		if (SPIHandle->pSPIx->CR1 & SPI_CR1_DFF) {
			// 16-bit data frame
			// load the data from data reg(DR) to RXBuffer addr
			*((uint16_t*) RX_BUFFER) = SPIHandle->pSPIx->DR; // Cast and dereference as 16-bit , loads 2 bytes of data into Rx Buffer
			LENGTH -= 2;                           // Decrease length by 2 bytes
			RX_BUFFER++;                    // Move buffer pointer by 2 bytes
		} else {
			// 8-bit data frame format
			*RX_BUFFER = (uint8_t) SPIHandle->pSPIx->DR; //Cast and Write/store 8-bit data to the data register
			LENGTH--;                               // Decrease length by 1 byte
			RX_BUFFER = (uint16_t*) (((uint8_t*) RX_BUFFER) + 1); // Move pointer by 1 byte                      // Move buffer pointer by 1 byte
		}
	}

}

/**
 * @brief  Pull the NSS line low (active).
 * @param  config : SPI_Config_t containing NSS pin info.
 * @retval None
 */
void SPI_NSS_Low(SPI_Config_t *config)
{
	GPIO_WriteToOutputPin(config->GPIO_NSS.pGPIOx,config->GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
}

/**
 * @brief  Pull the NSS line high (inactive).
 * @param  config : SPI_Config_t containing NSS pin info.
 * @retval None
 */

void SPI_NSS_High(SPI_Config_t *config)
{
	GPIO_WriteToOutputPin(config->GPIO_NSS.pGPIOx,config->GPIO_NSS.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
}

/**
 * @brief  Transmit a single 8/16-bit word as SPI Master using polling.
 * @note   Assumes full-duplex mode; a dummy read from DR clears any received data.
 *
 * Steps:
 * 1. Pull NSS low to activate the slave.
 * 2. Wait until TXE=1.
 * 3. Write data to SPI DR (starts the shift-out).
 * 4. Wait until TXE=1 and RXNE=1 (data finished shifting).
 * 5. Read DR to clear any received dummy data.
 * 6. Wait until BSY=0 to ensure no ongoing transmission.
 * 7. Pull NSS high to deactivate the slave.
 *
 * @param  config  : Pointer to SPI handle structure.
 * @param  tx_data : Data to transmit (8-bit if DFF=0, 16-bit if DFF=1).
 * @retval None
 */
void SPI_Master_Send_Byte(SPI_Handle_t *config, uint16_t tx_data)
{
    // 1) Pull NSS low
    SPI_NSS_Low(&config->SPIConfig);

    // 2) Wait until TXE=1 (transmit buffer empty)
    while (!(config->pSPIx->SR & _SPI_SR_TXE));

    // 3) Write data to the SPI data register (starts transmission)
    config->pSPIx->DR = tx_data;

    // 4) Wait until TXE=1 (shift register empty) and RXNE=1 (dummy data received)
    while (!(config->pSPIx->SR & _SPI_SR_TXE));
    while (!(config->pSPIx->SR & _SPI_SR_RXNE));

    // 5) Read DR to clear the received data (avoid overrun in full-duplex)
    (void)config->pSPIx->DR;

    // 6) Wait until BSY=0 (SPI not busy)
    while (config->pSPIx->SR & _SPI_SR_BSY);

    // 7) Pull NSS high to end the transaction
    SPI_NSS_High(&config->SPIConfig);
}

/**
 * @brief  Receive a single 16-bit word as SPI Master using polling
 * @note   Hardware NSS is toggled here for a single transaction
 *
 * The algorithm for this function is:
 * 1. Pull NSS low to select the slave.
 * 2. Wait until TXE=1 (SPI is ready to transmit).
 * 3. Write dummy data (0xFFFF) into DR to generate clock signals.
 * 4. Wait until TXE=1 again (shift register ready).
 * 5. Wait until RXNE=1 (data received from slave).
 * 6. Read the received 16-bit data from DR.
 * 7. Wait until BSY=0 to ensure no ongoing communication.
 * 8. Pull NSS high to deselect the slave.
 * 9. Return the received 16-bit data.
 *
 * @param  config: Pointer to SPI handle structure
 * @retval 16-bit word received from slave
 */
uint16_t SPI_Master_Receive_Byte(SPI_Handle_t *config)
{
    // 1) Pull NSS Low (select the slave)
    SPI_NSS_Low(&config->SPIConfig);

    // 2) Wait until the transmit buffer is empty (TXE=1)
    while (!(config->pSPIx->SR & _SPI_SR_TXE));

    // 3) Write a dummy value (0xFFFF) to DR to generate clock signals
    config->pSPIx->DR = 0xFFFF;

    // 4) Wait until TXE=1 again (ensures dummy data is in the shift register)
   // while (!(config->pSPIx->SR & _SPI_SR_TXE));

    // 5) Wait until the receive buffer is not empty (RXNE=1)
    while (!(config->pSPIx->SR & _SPI_SR_RXNE));

    // 6) Read the received 16-bit data from DR
    uint16_t recv_data = config->pSPIx->DR;

    // 7) Wait until BSY=0 to ensure no ongoing communication
    while (config->pSPIx->SR & _SPI_SR_BSY);

    // 8) Pull NSS High (deselect the slave)
    SPI_NSS_High(&config->SPIConfig);

    // 9) Return the received data
    return recv_data;
}

uint16_t SPI_Master_TRX_Byte(SPI_Handle_t *config, uint16_t tx_data)
{
    // 1) Pull NSS Low
    SPI_NSS_Low(&config->SPIConfig);

    volatile uint16_t temp = 0;

    // 2) Wait until TXE (Transmit Buffer Empty)
    while (!(config->pSPIx->SR & _SPI_SR_TXE));

    // 3) Write data to the SPI data register
    config->pSPIx->DR = tx_data;

    // 4) Wait again for TXE (shift register empty) and RXNE (received data ready)
    while (!(config->pSPIx->SR & _SPI_SR_TXE));
    while (!(config->pSPIx->SR & _SPI_SR_RXNE));

    // 5) Read received data
    temp = config->pSPIx->DR;

    // 6) Wait until BSY (Busy) flag is cleared
    while (config->pSPIx->SR & _SPI_SR_BSY);

    // 7) Pull NSS High (if this is a single transaction)
    SPI_NSS_High(&config->SPIConfig);

    return temp;
}

int8_t SPI_TRX_Buffer(SPI_Handle_t *config, uint16_t *TXBUFFER, uint16_t *RXBUFFER, uint16_t tx_length, uint16_t rx_length) {
    // Ensure tx_length and rx_length are equal
    if (tx_length != rx_length) {
        return -1; // Error: Buffer lengths must match
    }

    for (uint16_t i = 0; i < tx_length; i++) {
        // Send and receive data using SPI_TRX_Byte
    	RXBUFFER[i] = SPI_Master_TRX_Byte(config, TXBUFFER[i]);
    }

    return 1; // Success
}


