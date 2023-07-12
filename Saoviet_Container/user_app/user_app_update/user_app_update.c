
#include "user_app_update.h"
#include "user_define.h"


/*=============Function Static==============*/
static uint8_t _Entry_Update(uint8_t event);
static uint8_t _Update_Firmware(uint8_t event);
/*================ Struct =================*/
sEvent_struct         sEventAppUpdate [] =
{
  { _EVENT_ENTRY_UPDATE,           0, 0, 0,          _Entry_Update},    
  { _EVENT_UPDATE_FIRMWARE,        1, 0, 5,          _Update_Firmware}, 
};  

uint32_t Size_File=0;
uint16_t Length_Get=0;
uint8_t  CRC_File =0;
uint32_t Pos_FileFtp=0;
uint32_t address_exflash = FLASH_ADDR_FIRMWARE;
uint32_t sector_exflash  = FLASH_ADDR_FIRMWARE;

Struct_Update       sUpdate_Firmware=
{
    .Update   = 0,
    .Status   = 0,
    .CRC_Recv = 0,
    .Size_File= 0,
};

static uint8_t _Entry_Update (uint8_t event)
{
    return 1;
}

uint8_t aRead[7];
static uint8_t _Update_Firmware(uint8_t event)
{
    static uint8_t once_init=0;
    uint8_t aWrite[7];
    uint8_t TempCrc=0;
    if(sUpdate_Firmware.Update == 1)
    {
        sUpdate_Firmware.Update = 0;
        Size_File=0;
        Length_Get=0;
        CRC_File =0;
        Pos_FileFtp=0;
        address_exflash = FLASH_ADDR_FIRMWARE;
        sector_exflash  = FLASH_ADDR_FIRMWARE;
        if(once_init == 0)
        {
            fPushBlockSimStepToQueue(aSimStepBlockFtpInit, sizeof(aSimStepBlockFtpInit));
            once_init = 1;
        }
        fPushBlockSimStepToQueue(aSimStepBlockFtpGetSize, sizeof(aSimStepBlockFtpGetSize));
    }
    if(sUpdate_Firmware.Status == 1)
    {
        aWrite[0] = 0xAA;
        aWrite[1] = 0x05;
        aWrite[2] = sUpdate_Firmware.Size_File>>24;
        aWrite[3] = sUpdate_Firmware.Size_File>>16;
        aWrite[4] = sUpdate_Firmware.Size_File>>8;
        aWrite[5] = sUpdate_Firmware.Size_File;
        
        for (uint8_t i = 0; i < 6; i++)
        TempCrc ^= aWrite[i];
        aWrite[6] = TempCrc;
        
        eFlash_S25FL_Erase_Sector(FLASH_ADDR_UPDATE);
        eFlash_S25FL_BufferWrite(aWrite, FLASH_ADDR_UPDATE, 7);
        eFlash_S25FL_BufferRead(aRead, FLASH_ADDR_UPDATE, 7);
        sUpdate_Firmware.Status  = 0;
        NVIC_SystemReset();
    }
    fevent_enable(sEventAppUpdate, event);
    return 1;
}
/*================Function Module Sim ==============*/
void Get_Size_File_FTP(sData *uart_string)
{
    HAL_Delay(1);
    uint8_t i=0;
    uint32_t division=1;
    i = (*uart_string).Length_u16;
    while(i>0)
    {
        if((*uart_string).Data_a8[i]>='0' && (*uart_string).Data_a8[i]<='9')
        {
            break;
        }
        i--;
    }
    while(i>0)
    {
        if((*uart_string).Data_a8[i]>='0' && (*uart_string).Data_a8[i]<='9')
        {
            Size_File = Size_File + ((*uart_string).Data_a8[i] - 0x30)*division;
            division = division*10;
        }
        else
        {
            break;
        }
        i--;
    } 
    sUpdate_Firmware.Size_File = Size_File;
    fPushBlockSimStepToQueue(aSimStepBlockFtpRead, sizeof(aSimStepBlockFtpRead));
}
uint32_t count_file1=0;
uint32_t count_file2=0;

void Get_File1_FTP(sData *uart_string)
{
    count_file1++;
    uint8_t  aOFFSET1[10] = {0};
    sData    strOffset1 = {&aOFFSET1[0], 0};
    
    uint8_t  aOFFSET2[10] = {0};
    sData    strOffset2 = {&aOFFSET2[0], 0};
    
    if(Size_File >= 256)
    {
        Length_Get = 256;
        Size_File -= 256;
    }
    else if( Size_File > 0&& Size_File < 256)
    {
        Length_Get = Size_File;
        Size_File = 0;
    }
    
    //chuyen offset ra string de truyen vao sim
    Convert_Uint64_To_StringDec(&strOffset1, Pos_FileFtp, 0);
    //chuyen offset ra string de truyen vao sim
    Convert_Uint64_To_StringDec(&strOffset2, Length_Get, 0);

    Pos_FileFtp += Length_Get;

    
    Sim_Common_Send_AT_Cmd(&uart_sim, strOffset1.Data_a8, strOffset1.Length_u16,1000);
    
    Sim_Common_Send_AT_Cmd(&uart_sim, ",", 1,1000);

    Sim_Common_Send_AT_Cmd(&uart_sim, strOffset2.Data_a8, strOffset2.Length_u16,1000);
}

void Get_File2_FTP(sData *uart_string)
{
    count_file2++;
    sData File_Ftp=
    {
        .Length_u16=0,
    };
    HAL_Delay(50);
    uint16_t i=0;
    while(i < (*uart_string).Length_u16)
    {
        if((*uart_string).Data_a8[i++]=='C' && (*uart_string).Data_a8[i++]=='O' && (*uart_string).Data_a8[i++]=='N'
           && (*uart_string).Data_a8[i++]=='N'&& (*uart_string).Data_a8[i++]=='E'&& (*uart_string).Data_a8[i++]=='C'
           && (*uart_string).Data_a8[i++]=='T'&& (*uart_string).Data_a8[i++]==0x0D&& (*uart_string).Data_a8[i++]==0x0A)
        {
            break;
        }
    }
    File_Ftp.Data_a8 = uart_string->Data_a8 + i;
    File_Ftp.Length_u16 = Length_Get;
    
    if(address_exflash == sector_exflash)
    {
        eFlash_S25FL_Erase_Sector(sector_exflash);
        sector_exflash += 4096;
    }
    eFlash_S25FL_BufferWrite(File_Ftp.Data_a8, address_exflash, File_Ftp.Length_u16);
    address_exflash += File_Ftp.Length_u16;
    for (uint16_t i = 0; i < File_Ftp.Length_u16; i++)
    CRC_File ^= File_Ftp.Data_a8[i];
    
    if(Size_File != 0)
    {
        fPushBlockSimStepToQueue(aSimStepBlockFtpRead, sizeof(aSimStepBlockFtpRead));
    }
    else
    {
        if(CRC_File == sUpdate_Firmware.CRC_Recv)
        {
            sUpdate_Firmware.Status=1;
        }
    }
}
/*================ Function Handler =================*/

uint8_t AppUpdate_Task(void)
{
	uint8_t i = 0;
	uint8_t Result = false;

	for (i = 0; i < _EVENT_END_UPDATE; i++)
	{
		if (sEventAppUpdate[i].e_status == 1)
		{
            Result = true;

			if ((sEventAppUpdate[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppUpdate[i].e_systick)  >=  sEventAppUpdate[i].e_period))
			{
                sEventAppUpdate[i].e_status = 0;  //Disable event
				sEventAppUpdate[i].e_systick = HAL_GetTick();
				sEventAppUpdate[i].e_function_handler(i);
			}
		}
	}
    
	return Result;
}

