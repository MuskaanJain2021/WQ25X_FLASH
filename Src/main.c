#include <stm32f407xx.h>
#include <W25Q64FV_Driver_STM32F407VGT6.h>
#include "SYSTICK.h"
//#include <delay.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
int i = 0;

uint32_t W25Qxx_ID;
// #include <GPIO.h>

//void delay1(int de) {
//	while (de-- > 0) {
//		for (uint32_t i = 0; i < 800000; i++);
//		 // clk runs at (fclk/2 as set on sclk)8MHz ie 1.25us  if we do 1.25 x800000 =0.75 sec
//
//
//}
//}

void W25Qxx_CS_LOW() {
	GPIOD->BSRR = GPIO_BSRR_BR12;
}

void W25Qxx_CS_HIGH() {
	GPIOD->BSRR = GPIO_BSRR_BS12;
}

void LED_PASS(void) {
	GPIOD->BSRR = GPIO_BSRR_BS13;
	GPIOD->BSRR = GPIO_BSRR_BR14;
}
void LED_FAIL(void) {
	GPIOD->BSRR = GPIO_BSRR_BS14;
	GPIOD->BSRR = GPIO_BSRR_BR13;
}

void W25Qxx_CS_Pin_Init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;	   // Enable Clock for GPIOD
	GPIOD->MODER |= GPIO_MODER_MODE12_0;	   // Set PD12 as output
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12);// Set PD12 output in push pull configuration
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12; // Set Speed of PD12 as high speed
	GPIOD->BSRR = GPIO_BSRR_BS12;		// Set pin state high as initial state
}

/* Configure PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI) for SPI1 */
void SPI1_GPIO_Init(void) {
	/* 1) Enable GPIOA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//PD12 is chip select line
//	    /* ---- NSS = PA4 -> AF5 ---- */
//	    // Clear mode bits, set Alternate Function
//	    GPIOA->MODER   &= ~(3U << (4 * 2));       // Clear bits for pin 4
//	    GPIOA->MODER   |=  (2U << (4 * 2));       // Alternate function mode (2)
//	    // Output type: push-pull
//	    GPIOA->OTYPER  &= ~(1U << 4);
//	    // Speed: very high ( 2=High, 3=Very high )
//	    GPIOA->OSPEEDR &= ~(3U << (4 * 2));
//	    GPIOA->OSPEEDR |=  (3U << (4 * 2));
//	    // No pull-up/pull-down
//	    GPIOA->PUPDR   &= ~(3U << (4 * 2));
//	    // Alternate function AF5 for SPI1
//	    GPIOA->AFR[0]  &= ~(0xF << (4 * 4));
//	    GPIOA->AFR[0]  |=  (5U << (4 * 4));

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

void spi1_init() {

	SPI1_GPIO_Init();

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = 0;
	SPI1->CR1 |= SPI_CR1_MSTR;
	/*Set clock to FPCLK/2*/
	SPI1->CR1 &= ~(SPI_CR1_BR);
	//SPI1->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
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

//Full-duplex : send one byte, get one byte back.
 uint8_t SPI1_TxRx(uint8_t tx)
{
    // Wait TXE
    if (!TIMEOUT_LOOP(!(SPI1->SR & SPI_SR_TXE), 10000))
        HANDLE_SPI_TIMEOUT("TXE not set");

    // 8-bit write to DR
    *(uint8_t*)&SPI1->DR = tx;

    // Wait RXNE
    if (!TIMEOUT_LOOP(!(SPI1->SR & SPI_SR_RXNE), 10000))
        HANDLE_SPI_TIMEOUT("RXNE not set");

    // 8-bit read clears RXNE (prevents OVR)
    return *(uint8_t*)&SPI1->DR;
}

// "Write one byte" API: send and discard the returned byte.
void SPI1_MASTER_TRANSFER_BYTE(uint8_t data) {

	(void)SPI1_TxRx(data);

}

// If you prefer to wait explicitly elsewhere
int SPI1_WaitDone(void)
{
    if (!TIMEOUT_LOOP((SPI1->SR & SPI_SR_BSY), 10000))
        HANDLE_SPI_TIMEOUT("BSY not cleared");
}

//"Write buffer" API: stream bytes (discarding RX), and wait BSY once
void SPI1_MASTER_TRANSFER_BUFFER(const uint8_t *data, uint8_t size) {
	while (size--) {
		(void)SPI1_TxRx(*data++);
	}

	if(!TIMEOUT_LOOP((SPI1->SR && SPI_SR_BSY),10000))
	{
		HANDLE_SPI_TIMEOUT("BUSY not cleared");
	}
}


// Receive exactly one byte (clocks with 0xFF).
uint8_t SPI1_MASTER_RECEIVE_BYTE() {
	return SPI1_TxRx(0xFF);
}



//Receive N bytes (clocks with 0xFF for each byte).
void SPI1_MASTER_RECIEVE_BUFFER(uint8_t *data, uint32_t size) {
	if (!data || !size)return;
	while (size--) {*data++ = SPI1_TxRx(0xFF);}

    // End-of-frame fence: wait once before CS high
    if (!TIMEOUT_LOOP((SPI1->SR & SPI_SR_BSY), 10000))
        HANDLE_SPI_TIMEOUT("BSY not cleared");
}

int W25Qxx_WriteDisable(void) {
	int RetVal;
	// Assert CS and send Write Disable command (0x04)
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(0x04); // 0x04 is the Write Disable command
	W25Qxx_CS_HIGH();

	//delay1(500);
	// Wait until the Write Enable Latch is cleared.
	// Using generic status check function, check SR1 for the WEL bit.
	if (W25Qxx_CheckStatusBit(ReadSR1, SR_WEL_MASK)) {
		RetVal = -1; // If WEL is still set, return an error code.
	} else {
		RetVal = 1; // Write disable successful
	}

	return RetVal;
}

uint8_t W25Qxx_ReadStatusReg(uint8_t regCmd) {
	uint8_t status;
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BUFFER(&regCmd, 1); // Send the command (e.g., 0x05 for SR1, 0x35 for SR2)
	status = SPI1_MASTER_RECEIVE_BYTE();	 // Receive the status register byte
	W25Qxx_CS_HIGH();
	return status;
}
uint8_t W25Qxx_CheckStatusBit(uint8_t regCmd, uint8_t mask) {
	// Read the status register using the provided command
	uint8_t status = W25Qxx_ReadStatusReg(regCmd);

	// Return non-zero if any of the bits specified by mask are set
	return (status & mask);
}

uint32_t W25Qxx_READID(void) {
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
void W25Qxx_READ_DATA(uint32_t START_PAGE, uint8_t Offset,
		uint32_t NO_OF_BYTES_TO_BE_READ, uint8_t *DATA_BUFFER) {

	if (DATA_BUFFER == NULL)
		return; // invalid buffer pointer

	// Particular memory address MEM_ADDR
	uint32_t MEM_ADDR = (START_PAGE * FLASH_PAGE_SIZE + Offset);

	// Check calculated address is within the flash capacity
	if (MEM_ADDR + NO_OF_BYTES_TO_BE_READ > FLASH_TOTAL_BYTES)
		return; // out of bounds access

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000)) {
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
	while (NO_OF_BYTES_TO_BE_READ) {
		*DATA_BUFFER = SPI1_MASTER_RECEIVE_BYTE();
		DATA_BUFFER++;
		NO_OF_BYTES_TO_BE_READ--;
	}
	W25Qxx_CS_HIGH(); // pull the CS high
}

void W25Qxx_READ_MEMORY(ReadType type, uint32_t index, uint8_t offset,
		uint32_t bytes, uint8_t *buffer) {
	uint32_t StartPage = 0;
	uint32_t length = 0;
	switch (type) {
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
void W25Qxx_Reset() {
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(0x66);
	SPI1_MASTER_TRANSFER_BYTE(0x99);
	// delay(1);
	W25Qxx_CS_HIGH();
}
int W25Qxx_EnableFlash(void) {
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

void EraseSector4KB(uint32_t start_Addr) {
	W25Qxx_EnableFlash(); // WEL INSTRUCTION
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(Sector_Erase4KB);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 16) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 8) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE(start_Addr & 0xFF);
	W25Qxx_CS_HIGH();

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000)) {
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

void EraseSector32KB(uint32_t start_Addr) {
	W25Qxx_EnableFlash();  // Set WEL bit
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(Sector_Erase32KB);  // 0x52
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 16) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 8) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE(start_Addr & 0xFF);
	W25Qxx_CS_HIGH();

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 100000)) {
		HANDLE_SPI_TIMEOUT("BUSY after 32KB sector erase");
	}

	W25Qxx_WriteDisable();  // Always disable write after erase
}

void EraseSector64KB(uint32_t start_Addr) {
	W25Qxx_EnableFlash();  // Set WEL bit
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(Sector_Erase64KB);  // 0xD8
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 16) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE((start_Addr >> 8) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE(start_Addr & 0xFF);
	W25Qxx_CS_HIGH();

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 200000)) {
		HANDLE_SPI_TIMEOUT("BUSY after 64KB sector erase");
	}

	W25Qxx_WriteDisable();  // Protect against unintended writes
}

void EraseChip() {
	W25Qxx_EnableFlash();  // Set WEL bit
	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(Chip_Erase);  // 0xC7 or 0x60
	W25Qxx_CS_HIGH();

	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 3000000)) {
		HANDLE_SPI_TIMEOUT("BUSY after chip erase");
	}

	W25Qxx_WriteDisable();
}

FlashStatus_t W25Qxx_WritePageInRange(uint32_t pageIndex, uint16_t offset,
		const uint8_t *data, uint32_t len) {

	//1)Check Page boundary of 256 bytes only(0-255) in a page

	if (offset >= FLASH_PAGE_SIZE|| len == 0|| ((uint32_t)offset +(uint32_t) len ) > FLASH_PAGE_SIZE)
		return FLASH_ERR_INVALID_ADDR;

	// 2) Compute 24-bit byte address
	uint32_t addr = pageIndex * FLASH_PAGE_SIZE + offset;

	//3) Enable Write first WEL bit
	if (!W25Qxx_EnableFlash()) {
		HANDLE_SPI_TIMEOUT("WEL not set before WritePage");
		return FLASH_ERR_TIMEOUT;
	}
	// 4) Issue Page Program (0x02) + 24-bit address + payload

	W25Qxx_CS_LOW();
	SPI1_MASTER_TRANSFER_BYTE(WriteData);  // 0x02: Page Program
	SPI1_MASTER_TRANSFER_BYTE((addr >> 16) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE((addr >> 8) & 0xFF);
	SPI1_MASTER_TRANSFER_BYTE(addr & 0xFF);

	SPI1_MASTER_TRANSFER_BUFFER((uint8_t*) data, len); //send exactly len bytes (not whole page)
	W25Qxx_CS_HIGH();

	// Wait for BUSY to clear with timeout (loop while BUSY==1)
	if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK), 500000)) {
		HANDLE_SPI_TIMEOUT("WritePage BUSY Timeout");
	}

	// Optional safety: disable WEL
	W25Qxx_WriteDisable();
	return FLASH_OK;
}

//Program the bytes that differ (current !=target)
FlashStatus_t program_Page_From_Posx_PosY(uint32_t base,const uint8_t* src , uint32_t Bytes_To_be_written)
{
   while(Bytes_To_be_written)
   {
	   uint16_t offset = base/FLASH_PAGE_SIZE;
	   uint16_t Scope_Bytes_Left =  FLASH_PAGE_SIZE - offset;
	   uint16_t WriteSize = (Bytes_To_be_written < Scope_Bytes_Left)? Bytes_To_be_written : Scope_Bytes_Left;

	   FlashStatus_t st = W25Qxx_WritePageInRange(base/FLASH_PAGE_SIZE,offset,src,WriteSize);
	   if (st != FLASH_OK)return st;
	   base += WriteSize; src+=WriteSize;Bytes_To_be_written-=WriteSize;

   }
   return FLASH_OK;
}

//Page Program  must run till it completes page boundary then move to  the next page automatically to write rest bytes
FlashStatus_t Programming_Bytes_Differing(const uint8_t *curr, const uint8_t *target,uint32_t base ,uint8_t Bytes_To_be_written)
{
	uint32_t idx = 0;
	while(idx <  Bytes_To_be_written){


		//if Same Bytes
		while ( idx < Bytes_To_be_written  && curr[idx] == target[idx])idx++;
		if(idx == Bytes_To_be_written)break;


		//Store count of differing bytes
		 uint32_t  cnt = idx;
		 while ( idx < Bytes_To_be_written && curr[idx] != target[idx])idx++;

		 FlashStatus_t st = program_Page_From_Posx_PosY(base + cnt ,&target[cnt],idx - cnt);
	     if (st != FLASH_OK)return st;
	}
	return FLASH_OK;
}


FlashStatus_t W25Qxx_BulkWrite(uint32_t start_addr, const uint8_t *data,uint32_t length)
{
	if (data == NULL || length == 0)
		return FLASH_OK;
	//Check boundary
	if (((uint64_t) start_addr + (uint64_t) length > FLASH_TOTAL_BYTES)) {
		HANDLE_SPI_TIMEOUT("BulkWrite out of range");
		return FLASH_ERR_INVALID_ADDR;;
	}

	uint32_t addr = start_addr;
	const uint32_t end = start_addr + length;

	static uint8_t sector_buf[SECTOR_SIZE];  //To avoid on stack used static

	//To track last erased sector index to avoid duplicate erases
	//int32_t last_erased_sector = -1;

	while (addr < end) {
		uint32_t sector_base = (addr / SECTOR_SIZE) * SECTOR_SIZE;
		uint32_t sector_limit = sector_base + SECTOR_SIZE;
		uint32_t chunk_end = (end < sector_limit) ? end : sector_limit;
		uint32_t chunk_len = chunk_end - addr;
		uint32_t off_in_sec = addr - sector_base;

		// 1) Read current sector (simple & robust)
		uint32_t first_page = sector_base / FLASH_PAGE_SIZE;
		for (uint32_t p = 0; p < SECTOR_SIZE / FLASH_PAGE_SIZE; ++p) {
			W25Qxx_READ_DATA(first_page + p, 0, FLASH_PAGE_SIZE,
					&sector_buf[p * FLASH_PAGE_SIZE]);
		}

		// 2) Decide erase
		int need_erase = 0;
		for (uint32_t idx = 0; idx < chunk_len; ++idx) {
			if (!only_1_to_0(sector_buf[off_in_sec + idx], data[idx])) {
				need_erase = 1;
				break;
			}

		}

		FlashStatus_t st = FLASH_OK;
		if (!need_erase) {
			//No erase write page program caution is page bytes boundary checks
			st = Programming_Bytes_Differing(&sector_buf[off_in_sec], data,addr, chunk_len);
			if (st != FLASH_OK)
				return st;
		} else {
			//Erase:Erase once,Program only non FF ,merge
			for (uint32_t idx = 0; idx < chunk_len; ++idx)
				sector_buf[off_in_sec + idx] = data[idx];

			EraseSector4KB(sector_base);
			if (!TIMEOUT_LOOP(W25Qxx_CheckStatusBit(ReadSR1, SR_BUSY_MASK),
					3000000))
				return FLASH_ERR_TIMEOUT;
		// scan: does this page have any non-FF byte?
			for (uint32_t p = 0; p < SECTOR_SIZE; p += FLASH_PAGE_SIZE) {
				const uint8_t *page = &sector_buf[p];
				int any_FFs = 0;
				for (uint32_t j = 0; j < FLASH_PAGE_SIZE; ++j) {
					if (page[j] != 0xFF) {
						any_FFs = 1;
						break;
					}
				}
					if (!any_FFs)
						continue;




				//non FF's in a page
				uint32_t i = 0;
				while (i < FLASH_PAGE_SIZE)
				{
					while (i < FLASH_PAGE_SIZE  && page[i] == 0xFF)i++;
					if (i == FLASH_PAGE_SIZE)break;
					uint32_t yes_Not_FF_Found = i;
					while (i < FLASH_PAGE_SIZE  && page[i] != 0xFF)i++;


					st = W25Qxx_WritePageInRange((sector_base + p)/FLASH_PAGE_SIZE,(uint16_t) yes_Not_FF_Found, &page[yes_Not_FF_Found],(uint16_t)(i- yes_Not_FF_Found));
                    if (st != FLASH_OK)return st;

				}


			}
		}

		data += chunk_len;
		addr += chunk_len;
	}
	return FLASH_OK;
}

void LED_Init_PD13_PD14(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  //Enable clock of GPIOD

	GPIOD->MODER |= GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0;    // Output
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_14);   // Push-pull
	GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14; // High speed
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR13 | GPIO_PUPDR_PUPDR14); // No pull-up/pull-down

//	GPIOD->MODER   |= (1u<<(13*2)) | (1u<<(14*2));
//	GPIOD->OTYPER  &= ~((1u<<13)|(1u<<14));
//	GPIOD->OSPEEDR |= (3u<<(13*2)) | (3u<<(14*2));

}

int main(void)
{
	//delay_init(16000000);
	W25Qxx_CS_Pin_Init();
	//W25Qxx_CS_HIGH();
	// delay1(1);
	// delay1(1);
	spi1_init();
	LED_Init_PD13_PD14();
	// delay(1);

	W25Qxx_Reset();
	delay_ms(5);
	//delay1(2);
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
	delay_ms(1000);
	//To read 10 bytes from the 9th page:
	uint8_t pageDataSrc[10] = { 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
	uint8_t pageDatadest[10];
	int len = 10;
	W25Qxx_WritePageInRange(8u, 0, pageDataSrc, len);

	W25Qxx_READ_MEMORY(READ_TYPE_PAGE, 8u, 0, len, pageDatadest);
	if (memcmp(pageDataSrc, pageDatadest, len) == 0)
		//Turn on green led else turn on red led
		LED_PASS();
	else
		LED_FAIL();

//	EraseSector4KB(read_addr2); // Erase sector
//	// To read the entire 3rd sector (sector index = 2)-0x030000 to 0x03FFFF:
//	uint8_t sectorData[SECTOR_SIZE]={0};
//	W25Qxx_BulkWrite(read_addr2, sectorData,sizeof(sectorData));
//	W25Qxx_READ_MEMORY(READ_TYPE_SECTOR, 2, 0, SECTOR_SIZE, sectorData);

//	EraseSector64KB(read_addr3); // Erase sector
//	// To read the entire 2nd block (block index = 1)-0x010000 to 01FFFF:
//	uint8_t blockData[BLOCK_SIZE]={0};
//	W25Qxx_BulkWrite(read_addr3, blockData,sizeof(blockData));
//	W25Qxx_READ_MEMORY(READ_TYPE_BLOCK, 1, 0, BLOCK_SIZE, blockData);

	while(1){}
//	while (1) {
//		i++;
//		if (i > 250) {
//			i = 0;
//		}
	//}
	//return 0;

}
