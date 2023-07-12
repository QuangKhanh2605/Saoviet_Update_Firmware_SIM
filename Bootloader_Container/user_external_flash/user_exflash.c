#include "user_exflash.h"

uint32_t Cal_Time(uint32_t Millstone_Time, uint32_t Systick_now) 
{
	if (Systick_now < Millstone_Time)
		return (0xFFFFFFFF - Millstone_Time + Systick_now);
	return (Systick_now - Millstone_Time);
}	

uint8_t Check_Time_Out (uint32_t Millstone_Time, uint32_t Time_Period_ms)
{	
	if (Cal_Time(Millstone_Time, HAL_GetTick()) >= Time_Period_ms) 
        return 1;
	
	return 0;
}

void S25FL_ChipSelect(uint8_t State)
{
	uint16_t	i = 0;

	for (i = 0; i<1000; i++);

	if (State == LOW)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	for (i = 0; i<1000; i++);
}


uint8_t S25FL_Send_Byte(uint8_t byte)
{
	uint8_t retVal=0;

	HAL_SPI_TransmitReceive(&hspi2, &byte, &retVal, 1, 1000);  //hspi2 hSPI

	return retVal;
}


void S25FL_Send_Address (uint32_t address)
{
    /* Send SectorAddr high nibble address byte */
	S25FL_Send_Byte((address & 0xFF0000) >> 16);
	  /* Send SectorAddr medium nibble address byte */
	S25FL_Send_Byte((address & 0xFF00) >> 8);
	  /* Send SectorAddr low nibble address byte */
	S25FL_Send_Byte(address & 0xFF);
}

void eFlash_S25FL_Enable(void)
{
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);	
    
    /* Send "Write Enable" instruction */
    S25FL_Send_Byte(S25FL_WREN);
    
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);	
}


uint8_t eFlash_S25FL_Erase_Sector(uint32_t SectorAddr)
{
	uint8_t Retry = 0;
    /* Send write enable instruction */
	do{	
        eFlash_S25FL_Enable();
		if(eFlash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) == 1)
		{
            /* Sector Erase */ 
            /* Select the FLASH: Chip Select low */
            S25FL_ChipSelect(LOW);		
            /* Send Sector Erase instruction  */
            S25FL_Send_Byte(S25FL_SECTOR_ERASE);
            /* Send SectorAddr high nibble address byte */
            S25FL_Send_Byte((SectorAddr & 0xFF0000) >> 16);
            /* Send SectorAddr medium nibble address byte */
            S25FL_Send_Byte((SectorAddr & 0xFF00) >> 8);
            /* Send SectorAddr low nibble address byte */
            S25FL_Send_Byte(SectorAddr & 0xFF);
            /* Deselect the FLASH: Chip Select high */
            S25FL_ChipSelect(HIGH);
            /* Wait the end of Flash writing */
            if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_ER_SECTOR) == 1)
                return 1;
            else
                return 0;
		}
		Retry++;
	}while(Retry < 3);
    
	return 0;
}


uint8_t eFlash_S25FL_WaitForWriteEnd(uint32_t timeout) 
{
    uint8_t   FLASH_Status = 0;
    uint32_t	Start_check = 0;
    
    Start_check = HAL_GetTick();;
    
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);
  	
    /* Send "Read Status Register" instruction */
    S25FL_Send_Byte(S25FL_READ_STATUS);
    
    /* Loop as long as the memory is busy with a write cycle */ 		
    do
    { 
        /* Send a dummy byte to generate the clock needed by the FLASH 
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = S25FL_Send_Byte(DUMMY_BYTE); 
		
        if (Check_Time_Out(Start_check,timeout) == 1)
        {
            S25FL_ChipSelect(HIGH);
            return 0;
        }
        if ((FLASH_Status & WIP_Flag) != 0)                    //Bit so 0: WIP   ; Bit so 1: WEL:  1: cho phep program,,,, 0 K cho phep
            HAL_Delay(1);
    } while ((FLASH_Status & WIP_Flag) != 0);     /* Write in progress */  
    
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);
    
    return 1;
}



uint8_t eFlash_S25FL_CheckWriteEnable(uint32_t timeout)
{
    uint8_t     FLASH_Status = 0;
	uint32_t	Start_check = 0;
	
	Start_check = HAL_GetTick();
    
    /* Select the FLASH: Chip Select low */
    S25FL_ChipSelect(LOW);
  	
    /* Send "Read Status Register" instruction */
    S25FL_Send_Byte(S25FL_READ_STATUS);
    
    /* Loop as long as the memory is busy with a write cycle */ 		
    do
    { 
        /* Send a dummy byte to generate the clock needed by the FLASH 
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = S25FL_Send_Byte(DUMMY_BYTE); 
		
        if (Check_Time_Out(Start_check,timeout) == 1)
        {
            S25FL_ChipSelect(HIGH);
            return 0;
        }
        if((FLASH_Status & WEL_Flag) != WEL_Flag)   
            HAL_Delay(1);
    }while((FLASH_Status & WEL_Flag) != WEL_Flag); /* Write in progress */     //Neu 1: cho phep write    0: k cho phep
    /* Deselect the FLASH: Chip Select high */
    S25FL_ChipSelect(HIGH);
    
    return 1;
}



uint8_t eFlash_S25FL_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    //	uint8_t Retry = 0;
	uint8_t mTempReadback[256];
	
    //	do{
    /* Enable the write access to the FLASH */
    eFlash_S25FL_Enable(); // Check enable thanh cong
    if(eFlash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) == 1)
    {		
        /* Select the FLASH: Chip Select low */
        S25FL_ChipSelect(LOW);	   
        /* Send "Write to Memory " instruction */    
        S25FL_Send_Byte(S25FL_WRITE);	
        /* Send WriteAddr high nibble address byte to write to */
        S25FL_Send_Byte((WriteAddr & 0xFF0000) >> 16);
        /* Send WriteAddr medium nibble address byte to write to */
        S25FL_Send_Byte((WriteAddr & 0xFF00) >> 8);  
        /* Send WriteAddr low nibble address byte to write to */
        S25FL_Send_Byte(WriteAddr & 0xFF);             
        
        /* while there is data to be written on the FLASH */
        HAL_SPI_TransmitReceive(&hspi2, pBuffer, mTempReadback, NumByteToWrite, 100);
        
        //            /* Deselect the FLASH: Chip Select high */
        //            Retry = 3;
    } else
        return 0;
    
    S25FL_ChipSelect(HIGH);
    //		Retry++;
    //	}while(Retry<3);
    //    
    //    if(Flash_S25FL_CheckWriteEnable(TIMEOUT_WR_ENABLE) != 1)
    //		return 0;
    /* Wait the end of Flash writing */
    if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_PAGE) != 1)
        return 0;
    else
        return 1;
}


uint8_t eFlash_S25FL_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{  
    if (eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_ENABLE) == 1)
    {
        if (eFlash_S25FL_PageWrite(pBuffer, WriteAddr, NumByteToWrite) != 1)
        {
            return 0;
        }
    }
    
    return 1;
}

            

uint8_t eFlash_S25FL_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	uint8_t mAdress[3];
	uint16_t mNumByteToRead = NumByteToRead;
	uint8_t Retry = 0;
	
	mAdress[0] = (uint8_t)((ReadAddr & 0xFF0000) >> 16);
	mAdress[1] = (uint8_t)((ReadAddr & 0x00FF00) >> 8);
	mAdress[2] = (uint8_t)(ReadAddr & 0x0000FF);
	
	do
	{
		if(eFlash_S25FL_WaitForWriteEnd(TIMEOUT_WR_ENABLE) == 1)
		{
            /* Select the FLASH: Chip Select low */
            S25FL_ChipSelect(LOW);
            
            /* Send "Read from Memory " instruction */
            S25FL_Send_Byte(S25FL_READ);	 
            // Send addr
            /* Send ReadAddr high nibble address byte to read from */
            S25FL_Send_Byte(mAdress[0]);
            /* Send ReadAddr medium nibble address byte to read from */
            S25FL_Send_Byte(mAdress[1]);
            /* Send ReadAddr low nibble address byte to read from */
            S25FL_Send_Byte(mAdress[2]);   
			
            while (mNumByteToRead--) /* while there is data to be read */
            {
                /* Read a byte from the FLASH */			
                *pBuffer = S25FL_Send_Byte(DUMMY_BYTE);
                /* Point to the next location where the byte read will be saved */
                pBuffer++;
            }
            
            /* Deselect the FLASH: Chip Select high */
            S25FL_ChipSelect(HIGH);
            return 1;
        }
		Retry++;
	} while (Retry<3);
    
	return 0;
}