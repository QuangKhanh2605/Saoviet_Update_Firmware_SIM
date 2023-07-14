
#include "user_app_update.h"
#include "user_define.h"


/*=============Function Static==============*/
static uint8_t _Entry_Update(uint8_t event);
static uint8_t _Update_Firmware(uint8_t event);
static uint8_t _System_Reset(uint8_t event);
/*================ Struct =================*/
sEvent_struct         sEventAppUpdate [] =
{
  { _EVENT_ENTRY_UPDATE,           0, 0, 0,          _Entry_Update},    
  { _EVENT_UPDATE_FIRMWARE,        1, 0, 5,          _Update_Firmware}, 
  { _EVENT_SYSTEM_RESET,           0, 0, 10000,      _System_Reset},
};  

Struct_Param_Flash   sParam_ExFlash=
{
    .Addr   = FLASH_ADDR_FIRMWARE,
    .Sector = FLASH_ADDR_FIRMWARE,
};

Struct_Update       sUpdate_Firmware=
{
    .Update   = 0,
    .Status   = 0,
    .CRC_Recv = 0,
    .Size_File= 0,
};

Struct_Param_FTP    sParam_Update=
{
    .Size_File_u32     = 0,
    .Length_Handle_u16 = 0,
    .Crc_File_u16      = 0xFFFF,
    .Pos_File_u32      = 0,
};



static uint8_t _Entry_Update (uint8_t event)
{
    return 1;
}

static uint8_t _System_Reset(uint8_t event)
{
    static uint8_t handle = 0;
    handle++;
    if(handle == 2)
    {
        NVIC_SystemReset();
    }
    fevent_enable(sEventAppUpdate, event);
    return 1;
}

static uint8_t _Update_Firmware(uint8_t event)
{
    static uint8_t once_init=0;
    if(sUpdate_Firmware.Update == 1)
    {
        sUpdate_Firmware.Update = 0;
        if(once_init == 0)
        {
            fPushBlockSimStepToQueue(aSimStepBlockFtpInit, sizeof(aSimStepBlockFtpInit));
            once_init = 1;
        }
        fPushBlockSimStepToQueue(aSimStepBlockFtpGetSize, sizeof(aSimStepBlockFtpGetSize));
    }
    if(sUpdate_Firmware.Status == 1)
    {
        uint8_t TempCrc=0;
        uint8_t aWrite[7];
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
        DCU_Respond(2, (uint8_t*)"Download_Firmware_Success", 25, 0);
        fevent_active(sEventAppUpdate, _EVENT_SYSTEM_RESET);
        return 1;
    }
    fevent_enable(sEventAppUpdate, event);
    return 1;
}


/*================Function Module Sim ==============*/
void Get_Size_File_FTP(sData *uart_string)
{
    HAL_Delay(5);
    Reset_Update();
    if(strstr((char*)(*uart_string).Data_a8,"QFTPSIZE: 0") != NULL)
    {
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
                sParam_Update.Size_File_u32 = sParam_Update.Size_File_u32 + ((*uart_string).Data_a8[i] - 0x30)*division;
                division = division*10;
            }
            else
            {
                break;
            }
            i--;
        } 
        sUpdate_Firmware.Size_File = sParam_Update.Size_File_u32;
        fPushBlockSimStepToQueue(aSimStepBlockFtpRead, sizeof(aSimStepBlockFtpRead));
    }
    else
    {
        DCU_Respond(2, (uint8_t *)"Download_Firmware_Error_File", 28, 0);
    }
}

void Get_File1_FTP(sData *uart_string)
{
    uint8_t  aOFFSET1[10] = {0};
    sData    strOffset1 = {&aOFFSET1[0], 0};
    
    uint8_t  aOFFSET2[10] = {0};
    sData    strOffset2 = {&aOFFSET2[0], 0};
    
    if(sParam_Update.Size_File_u32 >= 256)
    {
        sParam_Update.Length_Handle_u16 = 256;
        sParam_Update.Size_File_u32    -= 256;
    }
    else if( sParam_Update.Size_File_u32 > 0&& sParam_Update.Size_File_u32 < 256)
    {
        sParam_Update.Length_Handle_u16 = sParam_Update.Size_File_u32;
        sParam_Update.Size_File_u32 = 0;
    }
    
    //chuyen offset ra string de truyen vao sim
    Convert_Uint64_To_StringDec(&strOffset1, sParam_Update.Pos_File_u32, 0);
    //chuyen offset ra string de truyen vao sim
    Convert_Uint64_To_StringDec(&strOffset2, sParam_Update.Length_Handle_u16, 0);

    sParam_Update.Pos_File_u32 += sParam_Update.Length_Handle_u16;
    
    Sim_Common_Send_AT_Cmd(&uart_sim, strOffset1.Data_a8, strOffset1.Length_u16,1000);
    
    Sim_Common_Send_AT_Cmd(&uart_sim, ",", 1,1000);

    Sim_Common_Send_AT_Cmd(&uart_sim, strOffset2.Data_a8, strOffset2.Length_u16,1000);
}

void Get_File2_FTP(sData *uart_string)
{
    sData File_Ftp=
    {
        .Length_u16=0,
    };
    uint16_t i=0;
    HAL_Delay(50);
    while(i < (*uart_string).Length_u16)
    {
        if((*uart_string).Data_a8[i++]=='C' && (*uart_string).Data_a8[i++]=='O' && (*uart_string).Data_a8[i++]=='N'
           && (*uart_string).Data_a8[i++]=='N'&& (*uart_string).Data_a8[i++]=='E'&& (*uart_string).Data_a8[i++]=='C'
           && (*uart_string).Data_a8[i++]=='T'&& (*uart_string).Data_a8[i++]==0x0D&& (*uart_string).Data_a8[i++]==0x0A)
        {
            break;
        }
    }

    if(sUpdate_Firmware.Status == 0)
    {
        File_Ftp.Data_a8 = uart_string->Data_a8 + i;
        File_Ftp.Length_u16 = sParam_Update.Length_Handle_u16;
        if(sParam_ExFlash.Addr == sParam_ExFlash.Sector)
        {
            eFlash_S25FL_Erase_Sector(sParam_ExFlash.Sector);
            sParam_ExFlash.Sector += 4096;
        }
        eFlash_S25FL_BufferWrite(File_Ftp.Data_a8, sParam_ExFlash.Addr, File_Ftp.Length_u16);
        sParam_ExFlash.Addr += File_Ftp.Length_u16;
            
        if(sParam_Update.Size_File_u32 > 0)
        {
            fPushBlockSimStepToQueue(aSimStepBlockFtpRead, sizeof(aSimStepBlockFtpRead));
        }
        else
        {
            uint32_t Addr   = FLASH_ADDR_FIRMWARE;
            uint32_t Size   = sUpdate_Firmware.Size_File;
            uint16_t Length = 0;
            while(Size > 0)
            {
              uint8_t aRead_ExFlash[256] = {0};
                if(Size >=256)
                {
                    Size -= 256;
                    Length= 256;
                }
                else 
                {
                    Length = Size;
                    Size   = 0;
                }
                eFlash_S25FL_BufferRead(aRead_ExFlash, Addr, Length);
                Addr+=256;
                Calculator_Crc_U16(&sParam_Update.Crc_File_u16, aRead_ExFlash, Length);
            }
          
            if(sParam_Update.Crc_File_u16 == sUpdate_Firmware.CRC_Recv)
            {
                sUpdate_Firmware.Status=1;
            }
            else
            {
                //sParam_Update.Crc_File_u16 = 0xFFFF;
                DCU_Respond(2, (uint8_t*)"Download_Firmware_Error_Crc", 27, 0);
            }
        }   
    }
}

void Reset_Update(void)
{
    sParam_Update.Size_File_u32     = 0;
    sParam_Update.Length_Handle_u16 = 0;
    sParam_Update.Crc_File_u16      = 0xFFFF;
    sParam_Update.Pos_File_u32      = 0;
    sParam_ExFlash.Addr             = FLASH_ADDR_FIRMWARE;
    sParam_ExFlash.Sector           = FLASH_ADDR_FIRMWARE;
    
    sUpdate_Firmware.Size_File      = 0;
}

uint16_t Calculator_Crc_U16(uint16_t *crc, uint8_t* buf, int len)
{
  //uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) 
  {
    *crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((*crc & 0x0001) != 0) {      // If the LSB is set
        *crc >>= 1;                    // Shift right and XOR 0xA001
        *crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        *crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return 1;  
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

