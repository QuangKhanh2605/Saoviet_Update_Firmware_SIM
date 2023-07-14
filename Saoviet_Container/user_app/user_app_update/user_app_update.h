


#ifndef USER_APP_UPDATE_H
#define USER_APP_UPDATE_H

#include "user_util.h"
#include "event_driven.h"
#include "string.h"

#define USING_APP_UPDATE

typedef enum
{
    _EVENT_ENTRY_UPDATE,
    _EVENT_UPDATE_FIRMWARE,
    _EVENT_SYSTEM_RESET,

    _EVENT_END_UPDATE,
}eKindEventUpdate;

typedef struct
{
    uint8_t  Update;
    uint8_t  Status;
    uint16_t  CRC_Recv;
    uint32_t  Size_File;
}Struct_Update;

typedef struct
{
    uint32_t Size_File_u32;
    uint16_t Length_Handle_u16;
    uint16_t  Crc_File_u16;
    uint32_t Pos_File_u32;
}Struct_Param_FTP;

typedef struct
{
    uint32_t Addr;
    uint32_t Sector;
}Struct_Param_Flash;


extern sEvent_struct sEventAppUpdate[];
extern Struct_Update sUpdate_Firmware;

/*==========Function============*/
uint8_t     AppUpdate_Task(void);
void        Get_Size_File_FTP(sData *uart_string);
void        Get_File1_FTP(sData *uart_string);
void        Get_File2_FTP(sData *uart_strign);
void        Reset_Update(void);
uint16_t    Calculator_Crc_U16(uint16_t *crc, uint8_t* buf, int len);
#endif
