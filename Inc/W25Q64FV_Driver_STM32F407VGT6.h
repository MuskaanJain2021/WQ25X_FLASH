/*
 * W25Q64FV_Driver_STM32F407VGT6.h
 *
 *  Created on: Feb 2, 2025
 *      Author: muska
 */

#ifndef W25Q64FV_DRIVER_STM32F407VGT6_H_
#define W25Q64FV_DRIVER_STM32F407VGT6_H_
//#include <SPI.h>
#include <stdio.h>
#include <stm32f4xx.h>
#define FLASH_PAGE_SIZE  256
#define SECTOR_SIZE   4096 //4KB
#define BLOCK_SIZE    65536 //64KB
#define FLASH_TOTAL_BYTES  (8 * 1024 * 1024)//8MB OR 64M BITS

#define JEDECID    0x9F



#define ReadSR1  0x05 //literal treated as integer
#define WriteSR 0x01

#define ReadSR2 0x35


#define ReadData 0x03
#define WriteData 0x02

#define WriteEnable 0x06
#define WriteDisable 0x04


#define Sector_Erase4KB 0x20
#define Sector_Erase32KB 0x52
#define Sector_Erase64KB 0xD8
#define Chip_Erase    0xC7

/*
 * Status Register 1 (SR1) – Bits 7..0:
 *
 * Bit 7 (SRP0): Status Register Protection bit 0.
 *   - Controls whether the status register bits are locked for writing.
 *
 * Bit 6 (SEC): Sector/Block Protect.
 *   - Determines if the protection applies to sectors (4KB) or blocks (64KB).
 *
 * Bit 5 (TB): Top/Bottom Protect.
 *   - Selects whether the protection applies to the top or bottom of the memory array.
 *
 * Bits 4, 3, 2 (BP2, BP1, BP0): Block Protection bits.
 *   - Combined, these bits define which region of the memory is write-protected.
 *
 * Bit 1 (WEL): Write Enable Latch.
 *   - Set by the Write Enable command (0x06) and cleared after a program/erase operation.
 *
 * Bit 0 (BUSY): Busy Flag.
 *   - Indicates whether the device is currently busy with a program or erase operation.
 */
#define SR_BUSY_MASK    0x01   // Bit 0
#define SR_WEL_MASK     0x02   // Bit 1
#define SR_BP0_MASK     0x04   // Bit 2
#define SR_BP1_MASK     0x08   // Bit 3
#define SR_BP2_MASK     0x10   // Bit 4
#define SR_TB_MASK      0x20   // Bit 5
#define SR_SEC_MASK     0x40   // Bit 6
#define SR_SRP0_MASK    0x80   // Bit 7


/*
 * Status Register 2 (SR2) – Bits 7..0:
 *
 * Bit 7 (SUS): Erase/Program Suspend Status.
 *   - Indicates if a program or erase operation has been suspended.
 *
 * Bit 6 (CMP): Complement Protect.
 *   - Used with protection bits to reverse the protected region.
 *
 * Bits 5, 4, 3 (LB3, LB2, LB1): Security Register Lock bits.
 *   - One-time programmable bits that, when set, permanently lock the security registers.
 *
 * Bit 2: Reserved.
 *
 * Bit 1 (QE): Quad Enable.
 *   - When set, the device operates in Quad SPI mode.
 *
 * Bit 0 (SRP1): Status Register Protection bit 1.
 *   - Works in conjunction with SRP0 (from SR1) to protect the status registers.
 */
#define SR2_SUS_MASK    0x80   // Bit 7
#define SR2_CMP_MASK    0x40   // Bit 6
#define SR2_LB3_MASK    0x20   // Bit 5
#define SR2_LB2_MASK    0x10   // Bit 4
#define SR2_LB1_MASK    0x08   // Bit 3
// Bit 2 is reserved.
#define SR2_QE_MASK     0x02   // Bit 1
#define SR2_SRP1_MASK   0x01   // Bit 0


#define read_addr1 0x000000

typedef enum {
	READ_TYPE_PAGE, READ_TYPE_SECTOR, READ_TYPE_BLOCK
} ReadType;

typedef enum {
    FLASH_OK = 0,
    FLASH_BUSY,
    FLASH_ERR_TIMEOUT,
    FLASH_ERR_WRITE,
    FLASH_ERR_INVALID_ADDR,
    FLASH_ERR_COMM
} FlashStatus_t;


#define TIMEOUT_LOOP(condition, timeout_limit)  \
    ({ \
        uint32_t __timeout = (timeout_limit); \
        while ((condition) && (--__timeout)); \
        (__timeout != 0); \
    })

#define HANDLE_SPI_TIMEOUT(label) \
    do { \
        printf("SPI Timeout at %s\n", label); \
        SPI1->CR1 &= ~SPI_CR1_SPE; \
        SPI1->CR1 |= SPI_CR1_SPE; \
        while(1); \
    } while(0)


//typedef struct W25Qx_typedef
//{
//	SPI_TypeDef *SPI_Port;
//	GPIO_TypeDef *NSS_Port_x;
//	uint8_t NSS_Pin_x;
//	bool SPI_Pin_x;
//	uint8_t Manufacturer_ID;
//	uint8_t Memory_decide_to_operate_on;
//	uint8_t Capacity;
//	uin8_t Unique_ID;
//
//}W25Qx_typedef;

///*Sector 4kb and block 32kb & 64kb erase*/
//enum Block
//{
//  BLOCK_4KB = 0X20,
//  BLOCK_32KB = 0X52,
//  BLOCK_64KB = 0XD8,
//};

void W25Qxx_CS_LOW();
void W25Qxx_CS_HIGH();
void W25Qxx_CS_Pin_Init();
//uint8_t W25Qxx_GetBusyStatus(void);
uint32_t W25Qxx_READID(void);
void W25Qxx_READ_DATA(uint32_t START_PAGE, uint8_t Offset,uint32_t NO_OF_BYTES_TO_BE_READ, uint8_t *DATA_BUFFER);
void W25Qxx_READ_MEMORY(ReadType type, uint32_t index, uint8_t offset,uint32_t bytes, uint8_t *buffer);
void W25Qxx_WritePage(uint32_t pageIndex, const uint8_t *data);
void W25Qxx_Reset();
int W25Qxx_EnableFlash(void);
int W25Qxx_WriteDisable(void);
uint8_t W25Qxx_CheckStatusBit(uint8_t regCmd, uint8_t mask);
uint8_t W25Qxx_ReadStatusReg(uint8_t regCmd);
void EraseSector4KB(uint32_t start_Addr);
//void W25Q64FV_PowerDown(void);
//void delay_5ms(void);


void W25Qxx_BulkWrite(uint32_t start_addr, const uint8_t *data, uint32_t length);
//uint8_t w25q64_read_id(SPI_Handle_t *spi);

#endif /* W25Q64FV_DRIVER_STM32F407VGT6_H_ */
