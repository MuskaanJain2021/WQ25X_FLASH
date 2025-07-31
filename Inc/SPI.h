/*
 * SPI.h
 *
 *  Created on: Jan 6, 2025
 *      Author: muska
 */

#ifndef SPI_H_
#define SPI_H_
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "GPIO.h"

#define _SPI_SR_RXNE  SPI_SR_RXNE
#define _SPI_SR_TXE   SPI_SR_TXE
#define _SPI_SR_CHSIDE  SPI_SR_CHSIDE
#define _SPI_SR_UDR     SPI_SR_UDR
#define _SPI_SR_CRCERR  SPI_SR_CRCERR
#define _SPI_SR_OVR     SPI_SR_OVR
#define _SPI_SR_MODF    SPI_SR_MODF
#define _SPI_SR_BSY     SPI_SR_BSY
#define _SPI_SR_FRE     SPI_SR_FRE

/*
 * Enums for SPI Configuration
 */

/* SPI Device Mode */
typedef enum {
	SPI_DEVICE_MODE_MASTER = 1, SPI_DEVICE_MODE_SLAVE = 0
} SPI_DeviceMode_t;

/* SPI Bus Configuration */
typedef enum {
	SPI_BUS_CONFIG_MASTER_FD = 1,
	SPI_BUS_CONFIG_SLAVE_FD,
	SPI_BUS_CONFIG_MASTER_HD,
	SPI_BUS_CONFIG_SLAVE_HD,
	SPI_BUS_CONFIG_SIMPLEX_MASTER_RXONLY,
	SPI_BUS_CONFIG_SIMPLEX_SLAVE_RXONLY,
	SPI_BUS_CONFIG_SIMPLEX_MASTER_TXONLY,
	SPI_BUS_CONFIG_SIMPLEX_SLAVE_TXONLY
} SPI_BusConfig_t;

/* SPI Clock Speed (Prescaler) */
typedef enum {
	SPI_SCLK_SPEED_DIV2 = 0,
	SPI_SCLK_SPEED_DIV4,
	SPI_SCLK_SPEED_DIV8,
	SPI_SCLK_SPEED_DIV16,
	SPI_SCLK_SPEED_DIV32,
	SPI_SCLK_SPEED_DIV64,
	SPI_SCLK_SPEED_DIV128,
	SPI_SCLK_SPEED_DIV256
} SPI_SclkSpeed_t;

/* SPI Data Frame Format */
typedef enum {
	SPI_DFF_8BITS = 0, SPI_DFF_16BITS
} SPI_DFF_t;

/* SPI Frame Format */
typedef enum {
	SPI_FRAME_FORMAT_MSB_FIRST = 0, SPI_FRAME_FORMAT_LSB_FIRST
} SPI_FrameFormat_t;

/* SPI Clock Polarity */
typedef enum {
	SPI_CPOL_LOW = 0, SPI_CPOL_HIGH
} SPI_CPOL_t;

/* SPI Clock Phase */
typedef enum {
	SPI_CPHA_LOW = 0, SPI_CPHA_HIGH
} SPI_CPHA_t;

/* SPI Software Slave Management */
typedef enum {
	SPI_SSM_DI = 0, SPI_SSM_EN
} SPI_SSM_t;

typedef enum
{
	SPI_SSOE_DI = 0, SPI_SSOE_EN
}SPI_SSOE_t;

typedef enum
{
	SPI_SSI_DI = 0, SPI_SSI_EN
}SPI_SSI_t;

typedef struct {
	/* SPI1 PIN MAPPINGS */
	struct _SPI1 {
		/* SPI1 CLK PIN MAPPING */
		struct CLK1 {
			uint8_t PA5; /**< Pin PA5 for SPI1 CLK. */
			uint8_t PB3; /**< Pin PB3 for SPI1 CLK. */
		} CLK1;

		/* SPI1 MOSI PIN MAPPING */
		struct MOSI1 {
			uint8_t PA7; /**< Pin PA7 for SPI1 MOSI. */
			uint8_t PB5; /**< Pin PB5 for SPI1 MOSI. */
		} MOSI1;

		/* SPI1 MISO PIN MAPPING */
		struct MISO1 {
			uint8_t PA6; /**< Pin PA6 for SPI1 MISO. */
			uint8_t PB4; /**< Pin PB4 for SPI1 MISO. */
		} MISO1;

	} _SPI1;

	/* SPI2 PIN MAPPINGS */
	struct _SPI2 {
		/* SPI2 CLK PIN MAPPING */
		struct CLK2 {
			uint16_t PB10; /**< Pin PB10 for SPI2 CLK. */
			uint16_t PB13; /**< Pin PB13 for SPI2 CLK. */
		} CLK2;

		/* SPI2 MOSI PIN MAPPING */
		struct MOSI2 {
			uint16_t PC3; /**< Pin PC3 for SPI2 MOSI. */
			uint16_t PB15; /**< Pin PB15 for SPI2 MOSI. */
		} MOSI2;

		/* SPI2 MISO PIN MAPPING */
		struct MISO2 {
			uint16_t PC2; /**< Pin PC2 for SPI2 MISO. */
			uint16_t PB14; /**< Pin PB14 for SPI2 MISO. */
		} MISO2;

	} _SPI2;

	/* SPI3 PIN MAPPINGS */
	struct _SPI3 {
		/* SPI3 CLK PIN MAPPING */
		struct CLK3 {
			uint16_t PC10; /**< Pin PC10 for SPI3 CLK. */
			uint16_t PB3; /**< Pin PB3 for SPI3 CLK. */
		} CLK3;

		/* SPI3 MOSI PIN MAPPING */
		struct MOSI3 {
			uint16_t PC12; /**< Pin PC12 for SPI3 MOSI. */
			uint16_t PB5; /**< Pin PB5 for SPI3 MOSI. */
		} MOSI3;

		/* SPI3 MISO PIN MAPPING */
		struct MISO3 {
			uint16_t PC11; /**< Pin PC11 for SPI3 MISO. */
			uint16_t PB4; /**< Pin PB4 for SPI3 MISO. */
		} MISO3;

	} _SPI3;

} SPI_Pin;

extern SPI_Pin spiPins;
typedef struct SPIx {
	SPI_TypeDef *_SPI1; /**< SPI1 base address. */
	SPI_TypeDef *_SPI2; /**< SPI2 base address. */
	SPI_TypeDef *_SPI3; /**< SPI3 base address. */
} SPIx;

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct {
	//uint8_t clock_pin; /**< Clock pin number. */
	//uint8_t mosi_pin; /**< MOSI pin number. */
	//uint8_t miso_pin; /**< MISO pin number. */
	//GPIO_TypeDef *NSS_Port; /**< Pointer to the GPIO port for NSS. */
	//uint8_t NSS_Pin; /**< NSS pin number. */
	GPIO_Handle_t GPIO_CLK; /**< GPIO configuration for Clock pin. */
	GPIO_Handle_t GPIO_MOSI; /**< GPIO configuration for MOSI pin. */
	GPIO_Handle_t GPIO_MISO; /**< GPIO configuration for MISO pin. */
	GPIO_Handle_t GPIO_NSS; /**< GPIO configuration for NSS pin. */

	uint16_t SPI_DeviceMode; /**< SPI mode (Master/Slave). */
	uint16_t SPI_BusConfig;/**< SPI bus config type (full-duplex, half-duplex, simplex). */
	uint16_t SPI_SclkSpeed;/**<Clock speed by clk generate by baudrate bits BR[2:0}>**/
	uint16_t SPI_DFF;/**< Data format (8-bit, 16-bit). */
	uint16_t SPI_FrameFormat; /**< Frame format (MSB first, LSB first).>**/
	uint16_t SPI_CPOL;/**< Clock polarity (CPOL).>**/
	uint16_t SPI_CPHA;/**< Clock phase (CPHA).>**/
	uint16_t SPI_SSM;/**<Software Slave Management>**/
    uint16_t SPI_SSOE;
    uint16_t SPI_SSI;
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_TypeDef *pSPIx; /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t *pRxBuffer; /* !< To store the app. Rx buffer address > */
	uint32_t TxLen; /* !< To store Tx len > */
	uint32_t RxLen; /* !< To store Tx len > */
	uint8_t TxState; /* !< To store Tx state > */
	uint8_t RxState; /* !< To store Rx state > */
} SPI_Handle_t;

uint8_t SPI_GetFlags_Status(SPI_TypeDef *pSPIx, uint32_t FlagName);
void SPI_Enable(SPI_Handle_t *config);
void SPI_Disable(SPI_Handle_t *config);
void SPI_NSS_Low(SPI_Config_t *config);
void SPI_NSS_High(SPI_Config_t *config);
int8_t SPI_Configuration_Reset(SPI_Handle_t *config);
void SPI_Data_Format_Update(SPI_Handle_t *config);
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);
void SPI_GPIO_Pin_Init(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF, uint8_t Speed);
void SPI_GPIO_Init1(SPI_Handle_t *config1);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *config);
//void SPI_SEND_DATA_POLLING(SPI_Handle_t *SPIHandle, uint8_t *TX_BUFFER,uint32_t LENGTH) ;
//void SPI_RECIEVE_DATA_POLLING (SPI_Handle_t *SPIHandle, uint16_t *RX_BUFFER, uint32_t LENGTH);
uint16_t SPI_Master_TRX_Byte(SPI_Config_t *config, uint16_t tx_data);
int8_t SPI_TRX_Buffer(SPI_Config_t *config, uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t tx_length, uint16_t rx_length);
void SPI_SendByte(SPI_TypeDef *SPIx, uint8_t data);
void  SPI_Master_Recieve_Byte();
void SPI_SSOE_Config(SPI_TypeDef *pSPIx, uint8_t SSOE_Status);
void SPI_SSI_Config(SPI_TypeDef *pSPIx, uint8_t SSI_Status);
#endif /* SPI_H_ */
