#ifndef __USER_EXTERNAL_FLASH
#define __USER_EXTERNAL_FLASH

#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "main.h"

#define FLASH_ADDR_FIRMWARE             0x7D1000
#define FLASH_ADDR_UPDATE               0x7D0000

#define SKIP				0x00

#define LOW    	 			0x00  /* Chip Select line low */
#define HIGH    			0x01  /* Chip Select line high */

#define S25FL_READ          0x03
#define S25FL_WRITE         0x02
#define S25FL_READ_STATUS   0x05
#define S25FL_WREN          0x06
#define S25FL_SECTOR_ERASE  0x20

#define S25FL_BLOCK_ERASE   0xD8
#define S25FL_CHIP_ERASE    0xC7
#define S25FL_JEDEC         0x9F  // 0x90
#define S25FL_WRITE_DISABLE 0x04

#define S25FL_VOLATILE_WREN 0x50
#define S25FL_RSTEN         0x66
#define S25FL_RST           0x99

#define WIP_Flag   		    0x03 // 0x01 vs 0x02 /* Write In Progress (WIP) flag */
#define WEL_Flag   		    0x02 /* Chua co check Flag WEL */
#define DUMMY_BYTE 		    0xA5

/* Define Size */
#define S25FL_PAGE_SIZE 	256
#define S25FL_SECTOR_SIZE 	4096

#define FLASH_S25FL_BASE 	0x0
#define FLASH_S25FL_END 	0x800000    //2048 sector



#define TIMEOUT_ER_SECTOR				1000
#define TIMEOUT_WR_PAGE					100
#define TIMEOUT_WR_ENABLE				1000
/*================Function===============*/
void S25FL_ChipSelect(uint8_t State);
uint8_t S25FL_Send_Byte(uint8_t byte);
void S25FL_Send_Address (uint32_t address);
void eFlash_S25FL_Enable(void);
uint8_t eFlash_S25FL_Erase_Sector(uint32_t SectorAddr);
uint8_t eFlash_S25FL_WaitForWriteEnd(uint32_t timeout);
uint8_t eFlash_S25FL_CheckWriteEnable(uint32_t timeout);
uint8_t eFlash_S25FL_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
uint8_t eFlash_S25FL_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
uint8_t eFlash_S25FL_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

uint32_t Cal_Time(uint32_t Millstone_Time, uint32_t Systick_now);
uint8_t Check_Time_Out (uint32_t Millstone_Time, uint32_t Time_Period_ms);

#endif
