


#ifndef USER_APP_UPDATE_H
#define USER_APP_UPDATE_H

#include "user_util.h"
#include "event_driven.h"

#define USING_APP_UPDATE

typedef enum
{
    _EVENT_ENTRY_UPDATE,
    _EVENT_UPDATE_FIRMWARE,

    _EVENT_END_UPDATE,
}eKindEventUpdate;

typedef struct
{
    uint8_t  Update;
    uint8_t  Status;
    uint8_t  CRC_Recv;
    uint32_t  Size_File;
}Struct_Update;


extern sEvent_struct sEventAppUpdate[];
extern Struct_Update sUpdate_Firmware;

/*==========Function============*/
uint8_t     AppUpdate_Task(void);
void        Get_Size_File_FTP(sData *uart_string);
void        Get_File1_FTP(sData *uart_string);
void        Get_File2_FTP(sData *uart_strign);

#endif
