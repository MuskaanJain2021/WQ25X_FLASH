#include <stm32f407xx.h>
#include <W25Q64FV_Driver_STM32F407VGT6.h>
#include <delay.h>
#include <stddef.h>
#include <stdint.h>
int i = 0;

uint32_t W25Qxx_ID;
// #include <GPIO.h>

void delay1(int de)
{
	while (de > 0)
	{
		for (uint32_t i = 0; i < 800000; i++) // clk runs at (fclk/2 as set on sclk)8MHz ie 1.25us  if we do 1.25 x800000 =0.75 sec
			de--;
	}
}

void W25Qxx_CS_LOW()
{
	GPIOD->BSRR = GPIO_BSRR_BR12;
}

void W25Qxx_CS_HIGH()
{
	GPIOD->BSRR = GPIO_BSRR_BS12;
}

void W25Qxx_CS_Pin_Init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;	   // Enable Clock for GPIOD
	GPIOD->MODER |= GPIO_MODER_MODE12_0;	   // Set PD12 as output
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12);	   // Set PD12 output in push pull configuration
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12; // Set Speed of PD12 as high speed
	GPIOD->BSRR = GPIO_BSRR_BS12;			   // Set pin state high as initial state
}

/* Configure PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI) for SPI1 */
void SPI1_GPIO_Init(void)
{
	/* 1) Enable GPIOA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//    /* ---- NSS = PA4 -> AF5 ---- */
	//    // Clear mode bits, set Alternate Function
	//    GPIOA->MODER   &= ~(3U << (4 * 2));       // Clear bits for pin 4
	//    GPIOA->MODER   |=  (2U << (4 * 2));       // Alternate function mode (2)
	//    // Output type: push-pull
	//    GPIOA->OTYPER  &= ~(1U << 4);
	//    // Speed: very high ( 2=High, 3=Very high )
	//    GPIOA->OSPEEDR &= ~(3U << (4 * 2));
	//    GPIOA->OSPEEDR |=  (3U << (4 * 2));
	//    // No pull-up/pull-down
	//    GPIOA->PUPDR   &= ~(3U << (4 * 2));
	//    // Alternate function AF5 for SPI1
	//    GPIOA->AFR[0]  &= ~(0xF << (4 * 4));
	//    GPIOA->AFR[0]  |=  (5U << (4 * 4));

	/* ---- SCK = PA5 -> AF5 ---- */
	GPIOA->MODER &= ~(3U << (5 * 2));
	GPIOA->MODER |= (2U << (5 * 2));
	GPIOA->OTYPER &= ~(1U << 5);
	GPIOA->OSPEEDR &= ~(3U << (5 * 2));
	GPIOA->OSPEEDR |= (3U << (5 * 2));
	GPIOA->PUPDR &= ~(3U << (5 * 2));
	GPIOA->AFR[0] &= ~(0xF << (5 * 4));
	GPIOA->AFR[0] |= (5U << (5 * 4));

	/* ---- MISO = PA6 -> AF5 ---- */
	GPIOA->MODER &= ~(3U << (6 * 2));
	GPIOA->MODER |= (2U << (6 * 2));
	GPIOA->OTYPER &= ~(1U << 6);
	GPIOA->OSPEEDR &= ~(3U << (6 * 2));
	GPIOA->OSPEEDR |= (3U << (6 * 2));
	GPIOA->PUPDR &= ~(3U << (6 * 2));
	GPIOA->AFR[0] &= ~(0xF << (6 * 4));
	GPIOA->AFR[0] |= (5U << (6 * 4));

	/* ---- MOSI = PA7 -> AF5 ---- */
	GPIOA->MODER &= ~(3U << (7 * 2));
	GPIOA->MODER |= (2U << (7 * 2));
	GPIOA->OTYPER &= ~(1U << 7);
	GPIOA->OSPEEDR &= ~(3U << (7 * 2));
	GPIOA->OSPEEDR |= (3U << (7 * 2));
	GPIOA->PUPDR &= ~(3U << (7 * 2));
	GPIOA->AFR[0] &= ~(0xF << (7 * 4));
	GPIOA->AFR[0] |= (5U << (7 * 4));
}

void spi1_init()
{

	SPI1_GPIO_Init();

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = 0;
	SPI1->CR1 |= SPI_CR1_MSTR;
	/*Set clock to FPCLK/2*/
	SPI1->CR1 &= ~(SPI_CR1_BR);
	/*Set CPOL =0 AND CPHA = 0 (MODE0)*/
	SPI1->CR1 &= ~SPI_CR1_CPOL;
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	// SPI IN FULL DUPLEX WE HAVEN'T USED SPI1->CR1 &= ~SPI1_CR1_RXONLYL

	/*Select Hardware Slave Management by selecting SSM =0 , SSOE =1*/
	SPI1->CR1 &= ~(SPI_CR1_SSM);
	SPI1->CR2 |= SPI_CR2_SSOE;
	// data format check

	/*Enable SPI module*/
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI1_MASTER_TRANSFER_BYTE(uint8_t data)
{
	SPI1->DR = data; // on writing data to data register, txe bit is cleared
	if (!TIMEOUT_LOOP(!(SPI1->SR & SPI_SR_TXE), 10000))//// when txe bit is set means data is shifted out from "out" pin
	{
		HANDLE_SPI_TIMEOUT("TXE not set");
	}

	if (!TIMEOUT_LOOP((SPI1->SR & SPI_SR_BSY), 10000))
	{
		HANDLE_SPI_TIMEOUT("BSY not cleared after TX");// wait for busy flag to reset
	}
	
}

void SPI1_MASTER_TRANSFER_BUFFER(uint8_t *data, uint8_t size)
{
	while (size > 0)
	{
		uint8_t TX_DATA = *data;
		SPI1_MASTER_TRANSFER_BYTE(TX_DATA);
		size--;
		data++;
	}
	/*Clear OVR flag*/
	(void)SPI1->DR;
	(void)SPI1->SR;
}

uint8_t SPI1_MASTER_RECEIVE_BYTE()
{
	uint8_t receive_byte = 0;
	SPI1->DR = 0xFF;
	if (!TIMEOUT_LOOP((SPI1->SR & SPI_SR_BSY), 10000))
	{
		HANDLE_SPI_TIMEOUT("BSY in RX");
	}

	if (!TIMEOUT_LOOP(!(SPI1->SR & SPI_SR_RXNE), 10000))
	{
		receive_byte = SPI1->DR;
		HANDLE_SPI_TIMEOUT("RXNE not set");
	}
	return receive_byte;
}

void SPI1_MASTER_RECIEVE_BUFFER(uint8_t *data, uint32_t size)
{
	while (size > 0)
	{
		*data++ = SPI1_MASTER_RECEIVE_BYTE();
		size--;
	}
}

int W25Qxx_WriteDisable(void)
{
	int RetVal;
	// Assert CS and send Write Disable command (0x04)
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(0x04); // 0x04 is the Write Disable command
	W25Qxx_CS_HIGH();

	delay1(500);
	// Wait until the Write Enable Latch is cleared.
	// Using generic status check function, check SR1 for the WEL bit.
	if (W25Qxx_CheckStatusBit(ReadSR1, SR_WEL_MASK))
	{
		RetVal = -1; // If WEL is still set, return an error code.
	}
	else
	{
		RetVal = 1; // Write disable successful
	}

	return RetVal;
}

uint8_t W25Qxx_ReadStatusReg(uint8_t regCmd)
{
	uint8_t status;
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BUFFER(&regCmd, 1); // Send the command (e.g., 0x05 for SR1, 0x35 for SR2)
	status = SPI1_MASTER_RECEIVE_BYTE();	 // Receive the status register byte
	W25Qxx_CS_HIGH();
	return status;
}
uint8_t W25Qxx_CheckStatusBit(uint8_t regCmd, uint8_t mask)
{
	// Read the status register using the provided command
	uint8_t status = W25Qxx_ReadStatusReg(regCmd);

	// Return non-zero if any of the bits specified by mask are set
	return (status & mask);
}

uint32_t W25Qxx_READID(void)
{
	uint8_t d[4];
	W25Qxx_CS_LOW();

	/*Read JEDEC*/
	SPI1_MASTER_TRANSFER_BYTE(JEDECID);
	d[0] = SPI1_MASTER_RECEIVE_BYTE();
	d[1] = SPI1_MASTER_RECEIVE_BYTE();
	d[2] = SPI1_MASTER_RECEIVE_BYTE();
	W25Qxx_CS_HIGH();
	return (d[0] << 16) | (d[1] << 8) | d[2];
}
void W25Qxx_READ_DATA(uint32_t START_PAGE, uint8_t Offset, uint32_t NO_OF_BYTES_TO_BE_READ, uint8_t *DATA_BUFFER)
{
	uint8_t retVal = 0, Retrys = 0xff;
	if (DATA_BUFFER == NULL)
		return; // invalid buffer pointer

	// Particular memory address MEM_ADDR
	uint32_t MEM_ADDR = (START_PAGE * FLASH_PAGE_SIZE + Offset);

	// Check calculated address is within the flash capacity
	if (MEM_ADDR + NO_OF_BYTES_TO_BE_READ > FLASH_TOTAL_BYTES)
		return; // out of bounds access

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000))
	{
		HANDLE_SPI_TIMEOUT("BUSY flag stuck during READ");
	}

	// Prepare the command to read data
	// The command is 0x03 followed by the 24-bit address
	// The address is split into three bytes: MSB, middle byte, and LSB

	uint8_t MEM_BYTE_WISE[4];
	MEM_BYTE_WISE[0] = ReadData;					// enable read;
	MEM_BYTE_WISE[1] = (MEM_ADDR >> 16) & 0xFF; // MSB of the memory Address
	MEM_BYTE_WISE[2] = (MEM_ADDR >> 8) & 0xFF;
	MEM_BYTE_WISE[3] = (MEM_ADDR) & 0xFF; // LSB of the memory address
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BUFFER(MEM_BYTE_WISE, 4); // send read instruction along with the 24 bit memory address
	while (NO_OF_BYTES_TO_BE_READ)
	{
		*DATA_BUFFER = SPI1_MASTER_RECEIVE_BYTE();
		DATA_BUFFER++;
		NO_OF_BYTES_TO_BE_READ--;
	}
	W25Qxx_CS_HIGH(); // pull the CS high
}

void W25Qxx_READ_MEMORY(ReadType type, uint32_t index, uint8_t offset, uint32_t bytes, uint8_t *buffer)
{
	uint32_t StartPage = 0;
	uint32_t length = 0;
	switch (type)
	{
	case READ_TYPE_PAGE:
		// For a page, the index is the page number and length is specified (<= FLASH_PAGE_SIZE)
		StartPage = index;
		length = bytes;
		break;

	case READ_TYPE_SECTOR:
		// For a sector, the index is the sector number.
		// Each sector is SECTOR_SIZE bytes, which equals (SECTOR_SIZE/FLASH_PAGE_SIZE) pages.
		StartPage = index * (SECTOR_SIZE / FLASH_PAGE_SIZE);
		length = bytes;
		// For sectors, offset is usually zero.
		break;

	case READ_TYPE_BLOCK:
		// For a block, the index is the block number.
		// Each block is BLOCK_SIZE bytes, which equals (BLOCK_SIZE/FLASH_PAGE_SIZE) pages.
		StartPage = index * (BLOCK_SIZE / FLASH_PAGE_SIZE);
		length = bytes;
		// For blocks, offset is usually zero.
		break;

	default:
		return; // Invalid type
	}

	// Call the low-level read function using the computed start page, offset, and length.
	W25Qxx_READ_DATA(StartPage, offset, length, buffer);
}
void W25Qxx_Reset()
{
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(0x66);
	SPI1_MASTER_TRANSFER_BYTE(0x99);
	// delay(1);
	W25Qxx_CS_HIGH();
}
int W25Qxx_EnableFlash(void)
{
	int RetVal;
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(WriteEnable);
	W25Qxx_CS_HIGH();
	if (W25Qxx_CheckStatusBit(ReadSR1, SR_WEL_MASK))
		RetVal = 1;
	else
		RetVal = -1;

	return RetVal;
}

// void W25Qxx_WriteData(uint32_t start_Addr, )
void EraseSector4KB(uint32_t start_Addr)
{
	W25Qxx_EnableFlash(); // WEL INSTRUCTION
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(Sector_Erase4KB);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 16) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 8) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE(start_Addr & 0xFF);
	W25Qxx_CS_HIGH();

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000))
	{
		HANDLE_SPI_TIMEOUT("BUSY after sector erase");
	}

	// while(W25Qxx_CheckStatusBit(ReadSR1,SR_BUSY_MASK)); //BSY flag if set flash busy in writing a program or erase operation
	// WEL Bit gets cleared in status register 1 that is device entered into write disable state
	// Sector erase won't occur if block protection bits in status register memory protection is enabled
	//  explicitly disable writes after erase.
	W25Qxx_WriteDisable();
	// Why Write disable explicitly needs to be called
	// guarantee that the flash is write-protected and no unintended write occurs.
	// Prevents any accidental writes that might occur if the Write Enable Latch remains set.
	// Ensures that the flash remains in a protected state until you explicitly enable writing for a new operation.
	// Even if the device automatically clears WEL, explicitly issuing a Write Disable command can serve as an extra safeguard in your firmware design.
}
void W25Qxx_WritePage(uint32_t pageIndex, const uint8_t *data)
{
    uint32_t addr = pageIndex * FLASH_PAGE_SIZE;

    // Enable Write first
    if (!W25Qxx_EnableFlash()) {
        HANDLE_SPI_TIMEOUT("WEL not set before WritePage");
        return;
    }

    W25Qxx_CS_LOW();
    SPI1_MASTER_TRANSFER_BYTE(WriteData);  // 0x02: Page Program
    SPI1_MASTER_TRANSFER_BYTE((addr >> 16) & 0xFF);
    SPI1_MASTER_TRANSFER_BYTE((addr >> 8) & 0xFF);
    SPI1_MASTER_TRANSFER_BYTE(addr & 0xFF);

    SPI1_MASTER_TRANSFER_BUFFER((uint8_t *)data, FLASH_PAGE_SIZE);
    W25Qxx_CS_HIGH();

    // Wait for BUSY to clear with timeout
    if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 500000)) {
        HANDLE_SPI_TIMEOUT("WritePage BUSY Timeout");
    }

    // Optional safety: disable WEL
    W25Qxx_WriteDisable();
}

void W25Qxx_BulkWrite(uint32_t start_addr, const uint8_t *data, uint32_t length)
{
	if (data == NULL || length == 0)
		return;

	uint32_t page_offset = start_addr % FLASH_PAGE_SIZE;
	uint32_t page_index = start_addr / FLASH_PAGE_SIZE;

	while (length > 0)
	{
		uint32_t current_addr = page_index * FLASH_PAGE_SIZE;
		uint32_t bytes_left_in_page = FLASH_PAGE_SIZE - page_offset;
		uint32_t write_size = (length < bytes_left_in_page) ? length : bytes_left_in_page;

		// Erase sector only at start of each 4KB
		if ((current_addr % SECTOR_SIZE) == 0)
		{
			EraseSector4KB(current_addr);
		}

		// Create a temporary page buffer
		uint8_t temp_page[FLASH_PAGE_SIZE];
		for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++)
			temp_page[i] = 0xFF;

		// Read-modify-write if it's a partial page
		if (page_offset > 0 || write_size < FLASH_PAGE_SIZE)
		{
			W25Qxx_READ_DATA(page_index, 0, FLASH_PAGE_SIZE, temp_page);
		}

		// Insert the incoming data into the temp page
		for (uint32_t i = 0; i < write_size; i++)
		{
			temp_page[page_offset + i] = data[i];
		}

		// Enable write and verify WEL bit before page program
		if (W25Qxx_EnableFlash() != 1)
		{
			// Handle error if WEL not set (e.g., return or log)
			return;
		}

		// Program the full page
		W25Qxx_WritePage(page_index, temp_page);

		if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000))
		{
			HANDLE_SPI_TIMEOUT("BUSY after page program");
		}

		// Optional: disable write
		W25Qxx_WriteDisable();

		// Advance pointers
		data += write_size;
		length -= write_size;
		page_index++;
		page_offset = 0;
	}
}

int main()
{
	delay_init(16000000);
	W25Qxx_CS_Pin_Init();
	W25Qxx_CS_HIGH();
	// delay1(1);
	// delay1(1);
	spi1_init();
	// delay(1);

	W25Qxx_Reset();
	delay(10);
	// delay1(1);
	// delay(1);
	/*Power release*/

	W25Qxx_ID = W25Qxx_READID();
	//	/*Slave Confirmation that I am alive to Master*/
	//	/*INSTRUCTION CODE : 90h*/
	//	SPI_MASTER_TRANSFER_BYTE(0x90);
	//	/*24 bit address A23-A0*/
	//	SPI_MASTER_TRANSFER_BYTE(0x00);
	//	SPI_MASTER_TRANSFER_BYTE(0x00);
	//	SPI_MASTER_TRANSFER_BYTE(0x00);
	//	SPI_MASTER_RECEIVE_BYTE();
	//	SPI_MASTER_RECEIVE_BYTE();
	//	cs_high();
	//--------------------------------------------
	//	/*READ PAGE*/
	//	/* PAGE HAS 256 BYTES(FLASH_TOTAL_BYTES )*/
	//	uint8_t PageData[128];
	//	/*Read the 9th page ie 10th physical page , bytes to be read from given page is 128Bytes*/
	//	W25Qxx_READ_DATA(9,0,128,PageData);
	//
	//
	//	/*READ SECTOR (0-15 INDEX)*/
	//	/*READ 3RD SECTOR=> SECTOR INDEX=2*/
	//	uint8_t SectorData[SECTOR_SIZE];
	//	uint32_t SectorIndex = 2;
	//
	//	//1 sector  has 4KB/256 = 16 pages
	//	uint32_t  StartPageForSector = SectorIndex * (SECTOR_SIZE/FLASH_PAGE_SIZE);//16 PAGES MAKES 4096 BYTES
	//	W25Qxx_READ_DATA(StartPageForSector,0,SECTOR_SIZE,SectorData);
	//
	//
	//	/*READ BLOCK*/
	//	/*READ 2nd BLOCK=> BLOCK INDEX=1*/
	//	uint8_t BlockData[BLOCK_SIZE];
	//	uint32_t BlockIndex = 1;
	//	//Each block has 65536 / 256 = 256 pages.
	//	uint32_t startPageForBlock = BlockIndex * (BLOCK_SIZE /FLASH_PAGE_SIZE);//256 PAGES MAKES 65536
	//	W25Qxx_READ_DATA(startPageForBlock, 0, BLOCK_SIZE, blockData);
	//-------------------------------------------

	EraseSector4KB(read_addr1); // Erase sector

	// To read 128 bytes from the 9th page:
	uint8_t pageData[128];
	W25Qxx_READ_MEMORY(READ_TYPE_PAGE, 9, 0, 128, pageData);

	// To read the entire 3rd sector (sector index = 2):
	uint8_t sectorData[SECTOR_SIZE];
	W25Qxx_READ_MEMORY(READ_TYPE_SECTOR, 2, 0, SECTOR_SIZE, sectorData);

	// To read the entire 2nd block (block index = 1):
	uint8_t blockData[BLOCK_SIZE];
	W25Qxx_READ_MEMORY(READ_TYPE_BLOCK, 1, 0, BLOCK_SIZE, blockData);

	while (1)
	{
		i++;
		if (i > 250)
		{
			i = 0;
		}
	}
}
