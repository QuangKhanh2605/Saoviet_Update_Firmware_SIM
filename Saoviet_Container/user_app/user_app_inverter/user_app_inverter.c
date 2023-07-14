/*
-   "at+ivtconfig=" Cau hinh Inverter (1 Byte)
    + (1)Config
    + (0) Stop Config
*Note: Khi cau hinh thi se tat trang thai dang ket noi Inverter qua RS485 (Khong
cho phep doc ghi du lieu tu dong hoac tu server gui xuong). Neu thay mat ket noi 
trong thoi gian dai can kiem tra thanh ghi cau hinh qua OBIS: 97.

-   "at+ivtcontrol=" Run Inverter (1 Byte)
    + (0) Stop and Lock(Run/Stop) Inverter
    + (1) Run and Unlock(Run/Stop) Inverter

*Note: Trang thai Unlock chi co tac dung tuc thoi khong cho dieu khien Inverter 
qua RS485. Neu can khoa Inverter khong cho dieu khien qua RS485 can cai dat cac
OBIS ben duoi cho phu hop.

-   "at+ivtread=" Doc gia tri thanh ghi duoc luu trong Flash (1 Byte)
    + (1) Addr Flash Default Inverter    (Flash: 0x0803F700)
    + (2) Addr Flash Run Inverter    (Flash: 0x0803F800)
    + (3) Addr Flash Config Inverter    (Flash: 0x0803F900)
    + (4) ALL
    + (5) Parameter:
        -Connect
        -Config
        -Permit Run/Stop 485
        -Lock Run/Stop 485

-   "at+ivtwriteD=" Default Inverter (6 Byte)
    + (1 Byte) ID Inverter (KDE200: 0x01)
    + (1 Byte) Scale Inverter (KDE200: 0xFF)

    Scale
    00->1 
    FF->0.1 
    FE->0.01 
    FD->0.001  
    FC->0.0001  
    FB->0.00001 

    + (2 Byte) Addr register setting frequency RS485 (KDE200: 0x03EB)
    + (2 Byte) Addr register output frequency (KDE200: 0x00DD)

-   "at+ivtwriteR=" Run Inverter RS485 (6 Byte)
    + (2 Byte) Addr register run Inverter (RS485) (KDE200: 0x00C8)
    + (2 Byte) Stop Inverter RS485 (KDE200: 0x0000)
    + (2 Byte) Run Inverter RS485 (KDE200: 0x0001)
*Note: Neu Stop = Run. Khong dieu khien Inverter qua RS485

-   "at+ivtwriteC=" Config register (6 Byte)
    + (1 Byte) Thu tu thanh ghi
    + (1 Byte) Tong thanh ghi can cau hinh
    + (2 Byte) Addr register
    + (2 Byte) Thong tin register can cau hinh
*Note: - Neu doc du lieu tu thanh ghi dau tien ma khong dung voi trang thai can
         cau hinh thi se thuc hien cau hinh lai (toi da cho phep 10 thanh ghi 
         can cau hinh)
       - Sau khi hoan thanh can kiem tra qua OBIS: 97 de xac nhan dung du lieu
         can cau hinh
       - Cac thanh ghi cau hinh can luu y cua KDE200:
         0x03E9, 0x03EA, 0x03EC, 0x03ED

*Note: Neu 3 lan gui data qua Inverter ma khong nhan duoc phan hoi => Mat ket noi 

*/
#include "user_app_inverter.h"
#include "user_define.h"

#include "user_modbus_rtu.h"
#include "user_sv_inverter.h"

/*================ Define =================*/

/*============= Function static  ===========*/
static uint8_t _Write_Read_IVT(uint8_t event);
static uint8_t _Config_IVT(uint8_t event);
static uint8_t _Entry_IVT(uint8_t event);
static uint8_t _Cb_Temh_Log_TSVH (uint8_t event);
static uint8_t _Handle_Recv_Server_IVT(uint8_t event);
/*================ Struct =================*/
sEvent_struct         sEventAppIVT [] =
{
  { _EVENT_ENTRY_IVT,           0, 0, 0,          _Entry_IVT},    
  
  { _EVENT_WRITE_FLASH,         1, 0, 50,         _Handle_Recv_Server_IVT},
  
  { _EVENT_CONFIG_IVT,          1, 0, 50, 	      _Config_IVT},
    
  { _EVENT_WRITE_READ_IVT,  	0, 0, 0,          _Write_Read_IVT},
	
  { _EVENT_TEMH_LOG_TSVH, 	    0, 0, 10000, 	  _Cb_Temh_Log_TSVH},
    
};      

Struct_Config_IVT      sConfig_IVT[] =
{
  { _ENUM_FREQUENCY_SETTING,           1, 1001, 0},
  
  { _ENUM_START_STOP_SETTING,          1, 1002, 3},
  
  { _ENUM_ACCELERATION_TIME,           1, 1004, 100},
  
  { _ENUM_DECELERATION_TIME,           1, 1005, 100},
};

Struct_Battery_Status    sBattery;
Struct_Battery_Status    sVout;

struct_ThresholdConfig   sMeterThreshold =
{
    .FlowHigh = 0xFFFF,
    .FlowLow = 0,
    .PeakHigh = 0xFFFF,
    .PeakLow = 0,
    .LowBatery = 10,
    .LevelHigh = 4000,
    .LevelLow  = 10,
};

uint8_t aMARK_ALARM[10];
uint8_t aMARK_ALARM_PENDING[10];

uint8_t  read_flash_IVT_u8 = 0;

uint8_t aDefault_IVT_Receive[6];
uint8_t aConfig_IVT_Receive[6];
uint8_t aRun_IVT_Receive[6];
uint8_t aConfig_IVT_Flash[NUMBER_BUFFER_CONFIG];

Param_Default_485_Inverter          sDefault_485_Inverter;
Param_Run_Stop_485_Inverter         sRun_Stop_485_Inverter;
Param_Check_Config_485_Inverter     sCheck_Config_485_Inverter;

Param_Inverter                      sParam_IVT=
{
    .Run_Stop_485 = 0xFF,
};

IVT_Receive                         sIVT_Recv=
{
    .sFreq.Status_Recv_u8=0,
    .sID_Server.Status_Recv_u8=0,
    
    .sDefault.Status_Recv_u8=0,
    .sDefault.aData = aDefault_IVT_Receive,
    
    .sRun.Status_Recv_u8=0,
    .sRun.aData = aRun_IVT_Receive,
    
    .sConfig.Status_Recv_u8=0,
    .sConfig.aData = aConfig_IVT_Receive,
};

STempHumiVariable         sTempHumi = 
{
    .aSlaveID = {SLAVE_ID_DEFAULT, SLAVE_ID_DEFAULT},
    .NumSlave_u8 = NUM_SLAVE_DEFAULT,
    .CountFailGetTemh_u8 = 1,
};

uint8_t aUART_485_DATA [64];
sData   sUart485 = {(uint8_t *) &aUART_485_DATA[0], 0};

static GPIO_TypeDef*  LED_PORT[3] = {LED_1_GPIO_Port, LED_2_GPIO_Port, LED_3_GPIO_Port};
static const uint16_t LED_PIN[3] = {LED_1_Pin, LED_2_Pin, LED_3_Pin};

/*================ Struct =================*/
static uint8_t _Entry_IVT (uint8_t event)
{
    fevent_active(sEventAppIVT, _EVENT_TEMH_LOG_TSVH);
    
    return 1;
}

/*
    Doc gia tri va thong tin thanh ghi Run/Stop Inverter
*/
uint8_t Setup_Addr_Register_Run_Stop_IVT(void)
{
    if(*(__IO uint8_t*)(ADDR_RUN_STOP_INVERTER) == BYTE_TEMP_FIRST)
    {
        sRun_Stop_485_Inverter.Addr_IVT = Read_Flash_U16(ADDR_RUN_STOP_INVERTER+2);
        sRun_Stop_485_Inverter.Stop_IVT = Read_Flash_U16(ADDR_RUN_STOP_INVERTER+4);
        sRun_Stop_485_Inverter.Run_IVT  = Read_Flash_U16(ADDR_RUN_STOP_INVERTER+6);
    }
    else
    {
        sRun_Stop_485_Inverter.Addr_IVT = DEF_ADDR_REGISTER_RUN_STOP;
        sRun_Stop_485_Inverter.Stop_IVT = DEF_INFOR_STOP;
        sRun_Stop_485_Inverter.Run_IVT  = DEF_INFOR_RUN;
    }
    
    if(sRun_Stop_485_Inverter.Run_IVT != sRun_Stop_485_Inverter.Stop_IVT )
    {
        sParam_IVT.Permit_Run_Stop_485 = 1;
    }
    else
    {
        sParam_IVT.Permit_Run_Stop_485 = 0;
    }
    
    return 1;
}

/*
    Doc gia tri thanh ghi Default Inverter
*/
uint8_t Setup_Addr_Register_Default_IVT(void)
{
    if(*(__IO uint8_t*)(ADDR_DEFAULT_INVERTER) == BYTE_TEMP_FIRST)
    {
        sDefault_485_Inverter.ID_Iverter     = *(__IO uint8_t*)(ADDR_DEFAULT_INVERTER +2);
        sDefault_485_Inverter.Scale_Inverter = *(__IO uint8_t*)(ADDR_DEFAULT_INVERTER +3);
        sDefault_485_Inverter.Set_Freq_IVT   = Read_Flash_U16(ADDR_DEFAULT_INVERTER +4);
        sDefault_485_Inverter.Out_Freq_IVT   = Read_Flash_U16(ADDR_DEFAULT_INVERTER +6);
    }
    else
    {
        sDefault_485_Inverter.ID_Iverter     = DEF_ID_INVERTER;
        sDefault_485_Inverter.Scale_Inverter = SCALE_IVT;
        sDefault_485_Inverter.Set_Freq_IVT   = DEF_ADDR_REGISTER_SETUP_FREQ;
        sDefault_485_Inverter.Out_Freq_IVT   = DEF_ADDR_REGISTER_OUT_FREQ;
    }

    return 1;
}

/*
    Doc gia tri thanh ghi Write Inverter
*/
uint8_t Setup_Addr_Register_Write_IVT(void)
{
    if(*(__IO uint8_t*)(ADDR_CONFIG_INVERTER) != BYTE_TEMP_FIRST)
    {
        UTIL_MEM_set(aConfig_IVT_Flash, 0x00, NUMBER_BUFFER_CONFIG);
        uint8_t j = 0;
        for(uint8_t i=0; i < _ENUM_FINISH_CONFIG;i++)
        {
          if(sConfig_IVT[i].status == 1)
          {
            aConfig_IVT_Flash[j]   = sConfig_IVT[i].address_register >> 8;
            aConfig_IVT_Flash[j+1] = sConfig_IVT[i].address_register;
            aConfig_IVT_Flash[j+2] = sConfig_IVT[i].infor_register >> 8;
            aConfig_IVT_Flash[j+3] = sConfig_IVT[i].infor_register;
            j+= 4;
          }
        }
        if(j%4 !=0)
        {
            j+= j%4;
        }
        Save_Array(ADDR_CONFIG_INVERTER, aConfig_IVT_Flash, j);
    }
    
    sCheck_Config_485_Inverter.Addr_IVT  = Read_Flash_U16(ADDR_CONFIG_INVERTER+2);
    sCheck_Config_485_Inverter.Infor_IVT = Read_Flash_U16(ADDR_CONFIG_INVERTER+4);
    
    return 1;
}

/*
    Function xu ly AT Command nhan duoc tu server
*/
static uint8_t _Handle_Recv_Server_IVT(uint8_t event)
{
/*----Xu ly hien thi array trong Flash len server----*/
  if(read_flash_IVT_u8 != 0)
  {
    if(read_flash_IVT_u8 <= 4)
    {
        Respond_Flash(2 , 0);
    }
    else if(read_flash_IVT_u8 == 5)
    {
        Respond_Parameter(2, 0);
    }
    read_flash_IVT_u8 = 0;
  }
  
/*----Xu ly chuoi nhan duoc tu server nap vao Flash (Default_485)---*/
  if(sIVT_Recv.sDefault.Status_Recv_u8 == 1)
  {
    sIVT_Recv.sDefault.Status_Recv_u8 = 0 ;
    Save_Array(ADDR_DEFAULT_INVERTER, sIVT_Recv.sDefault.aData, 6);
    Setup_Addr_Register_Default_IVT();
    DCU_Respond(2, sIVT_Recv.sDefault.aData, 6, 0);
  }
  
/*----Xu ly chuoi nhan duoc tu server nap vao Flash (Run_485)----*/
  if(sIVT_Recv.sRun.Status_Recv_u8 == 1)
  {
    sIVT_Recv.sRun.Status_Recv_u8 = 0;
    Save_Array(ADDR_RUN_STOP_INVERTER, sIVT_Recv.sRun.aData, 6);
    Setup_Addr_Register_Run_Stop_IVT();
    DCU_Respond(2, sIVT_Recv.sRun.aData, 6, 0);
  }
  
/*---- Xu ly chuoi nhan duoc tu server nap vao Flash (Config_485)---*/
  if(sIVT_Recv.sConfig.Status_Recv_u8 == 1)
  {
    sIVT_Recv.sConfig.Status_Recv_u8 = 0;
    if((sIVT_Recv.sConfig.aData[0] <= sIVT_Recv.sConfig.aData[1]) && sIVT_Recv.sConfig.aData[1] < NUMBER_CONFIG)
    {
        UTIL_MEM_set(aConfig_IVT_Flash, 0x00, NUMBER_BUFFER_CONFIG);
        uint8_t j=0;
        uint32_t address = ADDR_CONFIG_INVERTER + 2;
        for(uint8_t i=1 ; i <= sIVT_Recv.sConfig.aData[1]; i++)
        {
            if(i == sIVT_Recv.sConfig.aData[0])
            {
                aConfig_IVT_Flash[j]   = sIVT_Recv.sConfig.aData[2];
                aConfig_IVT_Flash[j+1] = sIVT_Recv.sConfig.aData[3];
                aConfig_IVT_Flash[j+2] = sIVT_Recv.sConfig.aData[4];
                aConfig_IVT_Flash[j+3] = sIVT_Recv.sConfig.aData[5];
            }
            else
            {
                aConfig_IVT_Flash[j]   = *(__IO uint8_t*)(address);
                aConfig_IVT_Flash[j+1] = *(__IO uint8_t*)(address+1);
                aConfig_IVT_Flash[j+2] = *(__IO uint8_t*)(address+2);
                aConfig_IVT_Flash[j+3] = *(__IO uint8_t*)(address+3);
            }
            j+= 4;
            address+= 4;
        }
        if(j%4 !=0)
        {
            j+= j%4;
        }
        Save_Array(ADDR_CONFIG_INVERTER, aConfig_IVT_Flash, j);
        DCU_Respond(2, aConfig_IVT_Flash, j, 0);
    }
  }
  
/*------ Phan hoi lai ID server va cac tham so -----*/
  if(sIVT_Recv.sID_Server.Status_Recv_u8 == 1)
  {
    sIVT_Recv.sID_Server.Status_Recv_u8 = 0;

    uint8_t     TempCrc = 0;  
    UTIL_MEM_set (aRESPONDSE_AT, 0, sizeof ( aRESPONDSE_AT ));
    sModem.strATResp.Length_u16 = 0;
    
    //Dong goi phan hoi
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = OBIS_TIME_DEVICE;   // sTime
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = 0x06;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.year;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.month;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.date;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.hour;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.min;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sRTC.sec;
    
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = OBIS_EMET_FREQ; 
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = 0x02;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (sIVT_Recv.sFreq.Pre_IVT >> 8) & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sIVT_Recv.sFreq.Pre_IVT & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sIVT_Recv.sFreq.Scale_IVT;
    
    
        //----------trang thai ket noi --------------------
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = OBIS_ENVI_CONNECT_STATUS; 
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = 0x01;
    //aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (connect_KDE >> 8) & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = sParam_IVT.Connect & 0xFF;
    //aRESPONDSE_AT[sModem.strATResp.Length_u16++] = 0x00;
    
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = OBIS_ENVI_ID_SERVER; 
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = 0x04;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (sIVT_Recv.sID_Server.ID_u32 >> 24) & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (sIVT_Recv.sID_Server.ID_u32 >> 16) & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (sIVT_Recv.sID_Server.ID_u32 >> 8) & 0xFF;
    aRESPONDSE_AT[sModem.strATResp.Length_u16++] = (sIVT_Recv.sID_Server.ID_u32) & 0xFF;
    
    // caculator crc
    sModem.strATResp.Length_u16++;
    for (uint8_t i = 0; i < (sModem.strATResp.Length_u16 - 1); i++)
        TempCrc ^= aRESPONDSE_AT[i];

    aRESPONDSE_AT[sModem.strATResp.Length_u16-1] = TempCrc;
    
    #ifdef USING_APP_SIM  
        sMQTT.aMARK_MESS_PENDING[SEND_RESPOND_SERVER] = TRUE;
    #endif
  }
  
    fevent_enable(sEventAppIVT, _EVENT_WRITE_FLASH);
    return 1;
}

/*
    Doc va phan hoi chuoi duoc save trong flash len server
*/
uint8_t Respond_Flash(uint8_t portNo, uint8_t Type)
{
  uint8_t aRespond_u8[55] = {0x00};
  uint16_t Length = 0;
  if(read_flash_IVT_u8 == 1 || read_flash_IVT_u8 == 4)
  {
      Read_Array_Flash(ADDR_DEFAULT_INVERTER, aRespond_u8, &Length);
  }
  
  if(read_flash_IVT_u8 == 2 || read_flash_IVT_u8 == 4)
  {
      Read_Array_Flash(ADDR_RUN_STOP_INVERTER, aRespond_u8, &Length);
  }

  if(read_flash_IVT_u8 == 3 || read_flash_IVT_u8 == 4)
  {
      Read_Array_Flash(ADDR_CONFIG_INVERTER, aRespond_u8, &Length);
  }
  
    DCU_Respond(portNo, aRespond_u8, Length, 0);
    return 1;
}

uint8_t Respond_Parameter(uint8_t portNo, uint8_t Type)
{
    uint8_t aRespond_u8[55] = {0x00};
    uint16_t Length = 0;
    aRespond_u8[Length++] = sParam_IVT.Connect;
    aRespond_u8[Length++] = sParam_IVT.Config;
    aRespond_u8[Length++] = sParam_IVT.Permit_Run_Stop_485;
    aRespond_u8[Length++] = sParam_IVT.Lock_Run_Stop_485;
    
    DCU_Respond(portNo, aRespond_u8, Length, 0);
    return 1;
}

/*
    Doc chuoi duoc luu trong Flash 
*/
uint8_t Read_Array_Flash(uint32_t Addr_Flash, uint8_t *aData, uint16_t *Location)
{
    aData[(*Location)++] = *(__IO uint8_t*)(Addr_Flash);
    aData[(*Location)++] = *(__IO uint8_t*)(Addr_Flash + 1);
      
    if(*(__IO uint8_t*)(Addr_Flash + 1) != 0)
    {
        for(uint8_t i = 1; i <= *(__IO uint8_t*)(Addr_Flash + 1); i++)
        {
            aData[(*Location)++] = *(__IO uint8_t*)(Addr_Flash + 1 + i);
        }
    }
    return 1;
}

/*
    Function doc va ghi gia tri vao Inverter
*/
static uint8_t _Write_Read_IVT(uint8_t event)
{ 
  static uint8_t step=0;
  static uint32_t gettick=0;
  static uint8_t count_disconnect=0;
  static uint8_t read_IVT=0;
  static uint8_t write_IVT=0;
  
    switch(step)
    {
        case 0:
          if(sIVT_Recv.sFreq.Status_Recv_u8 == 1)
          {
              uint16_t fre_send_IVT=0;
              fre_send_IVT = Calculator_Scale_IVT(sIVT_Recv.sFreq);
              Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x06, sDefault_485_Inverter.Set_Freq_IVT, fre_send_IVT,&AppIVT_Clear_Before_Recv);
              
              if(sParam_IVT.Permit_Run_Stop_485 == 1)
              {
                  if(sParam_IVT.Lock_Run_Stop_485 == 0)
                  {
                    sParam_IVT.Run_Stop_485 = sRun_Stop_485_Inverter.Run_IVT;
                  }
                  else
                  {
                    sParam_IVT.Run_Stop_485 = sRun_Stop_485_Inverter.Stop_IVT;
                  }
              }
              
              write_IVT = 0;
              step = 1;
              gettick = HAL_GetTick();
          }
          else if(sParam_IVT.Run_Stop_485 != 0xFF)
          {
            if(sParam_IVT.Permit_Run_Stop_485 == 1)
            {
                Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x06, sRun_Stop_485_Inverter.Addr_IVT, 
                                 sParam_IVT.Run_Stop_485,&AppIVT_Clear_Before_Recv);
                write_IVT = 1;
                step = 1;
                gettick = HAL_GetTick();
            }
            else
            {
                sParam_IVT.Run_Stop_485 = 0xFF;
            }
          }
          else
          {
            if(HAL_GetTick() - gettick > TIME_READ_FREQUENCY ) 
            {
              if(read_IVT == 0)
              {
                Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x03, sDefault_485_Inverter.Out_Freq_IVT, 1,&AppIVT_Clear_Before_Recv);
              }
              else
              {
                if(*(__IO uint8_t*)(ADDR_CONFIG_INVERTER + 1) != 0x00)
                {
                    Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x03, sCheck_Config_485_Inverter.Addr_IVT, 1,&AppIVT_Clear_Before_Recv);
                }
                else
                {
                    Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x03, sDefault_485_Inverter.Out_Freq_IVT, 1,&AppIVT_Clear_Before_Recv);
                    read_IVT = 0;
                }
              }
              step = 1;
              gettick = HAL_GetTick();
            }
          }
          break;
          
        case 1:
          if(HAL_GetTick() - gettick < TIME_RESEND_IVT)
          {
            if(Check_CountBuffer_Complete_Uart() == 1)
            {
                uint16_t CRC_Check = ModRTU_CRC(sUart485.Data_a8, sUart485.Length_u16-2);
                uint16_t CRC_Recv = sUart485.Data_a8[sUart485.Length_u16-1] << 8 | sUart485.Data_a8[sUart485.Length_u16-2];
                if(CRC_Check == CRC_Recv)
                {
                  if(sUart485.Data_a8[1] != 0x86 && sUart485.Data_a8[1] != 0x83)
                  {
                        if(sUart485.Data_a8[1] == 0x03)
                        {   
                              uint32_t Temp_u16 = 0;
                              for(uint8_t Length_Recv=sUart485.Data_a8[2]; Length_Recv > 0; Length_Recv--)
                              {
                                Temp_u16 = Temp_u16 << 8 | sUart485.Data_a8[sUart485.Length_u16- 2 -Length_Recv];
                              }
                              if(read_IVT == 0)
                              {
                                read_IVT = 1;
                                sParam_IVT.Response = Temp_u16;
                              }
                              else
                              {
                                read_IVT = 0;
                                if(Temp_u16 != sCheck_Config_485_Inverter.Infor_IVT)
                                {
                                    sParam_IVT.Config = 1;
                                }
                              }
                        }
                        else if(sUart485.Data_a8[1] == 0x06)
                        {
                              if(write_IVT == 0)
                              {
                                sIVT_Recv.sFreq.Status_Recv_u8 = 0;
                              }
                              else 
                              {
                                sParam_IVT.Run_Stop_485 = 0xFF;
                              }
                         }
                    }
                    else
                    {
                          if(write_IVT == 0)
                          {
                            sIVT_Recv.sFreq.Status_Recv_u8 = 0;
                          }
                          else
                          {
                            sParam_IVT.Run_Stop_485 = 0xFF;
                          }
                    }
                    step = 0;
                    sParam_IVT.Connect = 1;
                    count_disconnect = 0;
                }
                else
                {
                    step ++;
                    count_disconnect++;
                }
            }
          }
          else
          {
//              MX_USART1_UART_Init();
//              Init_Uart_485_Rx_IT();
              step = 0;
              count_disconnect++;
          }
          break;
          
        default: 
          if(HAL_GetTick() - gettick > TIME_RESEND_IVT)
          {
              step=0;
          }
          break;
    }
    
    if(count_disconnect >= 3)
    {
      sParam_IVT.Connect = 0;
      sParam_IVT.Response = 0;
    }
    //fevent_enable(sEventAppIVT, event);
    return 1;
}

void Toggle_Led(void)
{
   static uint32_t gettick=0;
   if(HAL_GetTick() - gettick  > 1000)
   {
        LED_Toggle(_LED_GPS);
        gettick = HAL_GetTick();
   }
}

/*
    Cau hinh Inverter
*/
static uint8_t _Config_IVT(uint8_t event)
{
  Toggle_Led();
  static uint8_t handle_once=0;
  static uint32_t gettick_config=0;
  if(sParam_IVT.Config == 1)
  {
    if(sIVT_Recv.sDefault.Status_Recv_u8 == 0 && sIVT_Recv.sConfig.Status_Recv_u8 == 0 
       && sIVT_Recv.sRun.Status_Recv_u8 == 0)
    {
        if(handle_once == 0)
        {
            Task_ConfigIVT(sDefault_485_Inverter.ID_Iverter, 1);
            fevent_disable(sEventAppIVT, _EVENT_WRITE_READ_IVT);
            gettick_config = HAL_GetTick();
            handle_once = 1;
        }
        
        if(HAL_GetTick() - gettick_config < TIME_SEND_OFF_IVT)
        {
//            sParam_IVT.Connect =0;
            if(Task_ConfigIVT(sDefault_485_Inverter.ID_Iverter, 0 ) == 1)
            {
                sCheck_Config_485_Inverter.Addr_IVT  = Read_Flash_U16(ADDR_CONFIG_INVERTER+2);
                sCheck_Config_485_Inverter.Infor_IVT = Read_Flash_U16(ADDR_CONFIG_INVERTER+4);

                sParam_IVT.Config = 0;
//                sParam_IVT.Connect = 1;
                UTIL_Printf_Str( DBLEVEL_M, "u_app_IVT200: Success Config IVT\r\n" );
            }
        }
        else
        {
            handle_once = 0;
            if(sParam_IVT.Permit_Run_Stop_485 == 1)
            {
                Send_Command_IVT(sDefault_485_Inverter.ID_Iverter, 0x06, sRun_Stop_485_Inverter.Addr_IVT,
                                 sRun_Stop_485_Inverter.Stop_IVT, &AppIVT_Clear_Before_Recv);
            }
        }
    }
  }
  else
  {
    handle_once=0;
    fevent_active(sEventAppIVT, _EVENT_WRITE_READ_IVT);
  }
  fevent_enable(sEventAppIVT, event);
  return 1;
}

/*
    Function Handle Config Inverter
*/
uint8_t Task_ConfigIVT(uint8_t ID_Slave, uint8_t Status_Reset)
{
    static uint8_t step=0;
    static uint8_t handle_once=0;
    static uint16_t addr_register = 0;
    static uint16_t infor_register = 0;
    uint32_t address = ADDR_CONFIG_INVERTER + 2;
    
    if(Status_Reset == 1)
    {
        step = 0;
        handle_once = 0;
        return 0;
    }
    else
    {
        if(step == (*(__IO uint8_t*)(ADDR_CONFIG_INVERTER + 1))/4)
        {
            step = 0;
            handle_once = 0;
            return 1;
        }
        else
        {
          if(handle_once == 0)
          {
            handle_once = 1;
            addr_register  = Read_Flash_U16(address + step*4);                  
            infor_register = Read_Flash_U16(address + step*4 + 2);
          }
          if(handle_once == 1)
          {
              if(Write_485_Complete_Command_IVT(ID_Slave, addr_register, infor_register) == 1) 
              {
                handle_once = 0;
                step++;
              }
          }
        }
    }

    return 0;
}

/*================ Function Handler =================*/

uint8_t AppIVT_Task(void)
{
	uint8_t i = 0;
	uint8_t Result = false;

	for (i = 0; i < _EVENT_END_IVT; i++)
	{
		if (sEventAppIVT[i].e_status == 1)
		{
            Result = true;

			if ((sEventAppIVT[i].e_systick == 0) ||
					((HAL_GetTick() - sEventAppIVT[i].e_systick)  >=  sEventAppIVT[i].e_period))
			{
                sEventAppIVT[i].e_status = 0;  //Disable event
				sEventAppIVT[i].e_systick = HAL_GetTick();
				sEventAppIVT[i].e_function_handler(i);
			}
		}
	}
    
	return Result;
}

/*
  @brief  Gui lenh toi IVT va doi phan hoi 
  @retval (0) Chua hoan thanh
  @retval (1) Da hoan thanh 
*/
uint8_t Write_485_Complete_Command_IVT(uint8_t Addr_Slave, uint16_t Addr_Register, uint16_t Infor_Register)
{
  static uint8_t step=0;
  static uint32_t gettick = 0;
  static uint8_t count_disconnect=0;
  uint8_t answer=0;
  
  switch (step)
  {
      case 0:
        Send_Command_IVT(Addr_Slave, 0x06, Addr_Register, Infor_Register ,&AppIVT_Clear_Before_Recv);
        gettick = HAL_GetTick();
        step++;
        break;
        
      case 1:
        if(HAL_GetTick() - gettick < 100)
        {
          if(Check_CountBuffer_Complete_Uart() == 1)
          {
            uint16_t CRC_Receive = ModRTU_CRC(sUart485.Data_a8, sUart485.Length_u16-2);
            uint16_t CRC_Check   = sUart485.Data_a8[sUart485.Length_u16-1]<<8 | sUart485.Data_a8[sUart485.Length_u16-2];
            uint16_t Addr_Register_Recv = sUart485.Data_a8[sUart485.Length_u16-6]<<8 | sUart485.Data_a8[sUart485.Length_u16-5];
            if(CRC_Check == CRC_Receive)
            {
              step++;
              if(Addr_Slave == sUart485.Data_a8[0])
              {
                if(sUart485.Data_a8[1] == 0x06)
                {
                    if(Addr_Register == Addr_Register_Recv)
                    {
                        answer = 1;
                        step = 0;
                    }
                }
              }
              sParam_IVT.Connect = 1;
              count_disconnect = 0;
            }
            else
            {
                step++;
                count_disconnect++;
            }
          }
        }
        else
        {
//          MX_USART1_UART_Init();
//          Init_Uart_485_Rx_IT();
          step = 0;
          count_disconnect++;
        }
        break;
        
      default:
        if(HAL_GetTick() - gettick > TIME_RESEND_IVT)
        {
            step=0;
        }
        break;
  }
  
    if(count_disconnect >= 3)
    {
      sParam_IVT.Connect = 0;
      sParam_IVT.Response = 0;
    }
  
  return answer;
}

/*
    Doc gia tri Uint16 tu Flash
*/
uint16_t Read_Flash_U16(uint32_t address)
{
    uint32_t result = 0 ;
    result = *(__IO uint8_t*)(address );
    result = result << 8 | *(__IO uint8_t*)(address +1);    
    return result;
}

/*
    Chuyen Scale sang Uint
*/
uint32_t Calculator_Scale(uint8_t Scale)
{
    /*
    Scale
    00->1 
    FF->0.1 
    FE->0.01 
    FD->0.001  
    FC->0.0001  
    FB->0.00001 
    */
  
    if(Scale == 0x00) return 1;
    else if(Scale == 0xFF) return 10;
    else if(Scale == 0xFE) return 100;
    else if(Scale == 0xFD) return 1000;
    else if(Scale == 0xFC) return 10000;
    else if(Scale == 0xFB) return 100000;
    
    return 1;
}

/*
    @brief  Tinh tan so theo Scale
    @retval Tan so nap vao bien tan
*/
uint16_t Calculator_Scale_IVT(IVT_Receive_Freq  sFre_IVT_Receive)
{
      /*
    Scale
    00->1 
    FF->0.1 
    FE->0.01 
    FD->0.001  
    FC->0.0001  
    FB->0.00001 
    */
  uint32_t Stamp_Scale = sFre_IVT_Receive.Pre_IVT * Calculator_Scale(sDefault_485_Inverter.Scale_Inverter);
  return Stamp_Scale/Calculator_Scale(sFre_IVT_Receive.Scale_IVT);
}

/*
	@brief  Kiem tra da nhan xong Uart hay chua 
	@param  sUart Struct uart muon kiem tra
	@return (0) chua nhan xong
	@return (1) da nhan xong
*/
uint8_t Check_CountBuffer_Complete_Uart(void)
{
  static uint16_t countBuffer_uart = 0;
  static uint32_t Get_systick = 0;
  static uint16_t State_systick = 0;
  uint8_t answer=0;
  
  if(sUart485.Length_u16 > 0)
  {
      if(State_systick == 0 )
      {
          Get_systick = HAL_GetTick();
          countBuffer_uart = sUart485.Length_u16;
          State_systick = 1;
      }
      else
      {
          if(HAL_GetTick() - Get_systick > 5)	
          {
              if(countBuffer_uart == sUart485.Length_u16)
              {
                  answer = 1;
              }
              else
              {
                  Get_systick = HAL_GetTick();
                  countBuffer_uart = sUart485.Length_u16;
              }
          }
          else
          {
              if(countBuffer_uart != sUart485.Length_u16)
              {
                  Get_systick = HAL_GetTick();
                  countBuffer_uart = sUart485.Length_u16;
              }
          }
      }
  }
  else
  {
      State_systick = 0;
  }
  return answer;
}

uint16_t AppTemH_Get_VBAT_mV(void)
{
    //Get batterry
    sBattery.mVol_u32 = Get_Value_ADC (ADC_CHANNEL_14);
    sBattery.mVol_u32 *= 3;   //Phan ap chia 2
    sBattery.Level_u16 = Get_Level_Voltage (sBattery.mVol_u32, VDD_BAT, VDD_MIN); 

	return sBattery.Level_u16;
}

uint16_t AppTemH_Get_Vout_mV(void)
{
    AdcInitialized = 0; 
    sVout.mVol_u32 = Get_Value_ADC(ADC_CHANNEL_7); 
    sVout.mVol_u32 = sVout.mVol_u32 * 247 / 47 + 600;  //qua 2 diode
    sVout.Level_u16 = Get_Level_Voltage (sVout.mVol_u32, VDD_OUT_MAX, VDD_OUT_MIN); 
    
	return sVout.Level_u16;
}

uint8_t AppIVT_Packet_TSVH (uint8_t *pData)
{
    uint8_t     length = 0;
    uint16_t	i = 0;
    uint8_t     TempCrc = 0;   
    //----------------------- start send data below --------------------- //
    //----------sTime--------------------
    pData[length++] = OBIS_TIME_DEVICE;   // sTime
    pData[length++] = 0x06;
    pData[length++] = sRTC.year;
    pData[length++] = sRTC.month;
    pData[length++] = sRTC.date;
    pData[length++] = sRTC.hour;
    pData[length++] = sRTC.min;
    pData[length++] = sRTC.sec;
    
    pData[length++] = OBIS_EMET_FREQ; 
    pData[length++] = 0x02;
    pData[length++] = (sParam_IVT.Response >> 8) & 0xFF;
    pData[length++] = sParam_IVT.Response & 0xFF;
    pData[length++] = sDefault_485_Inverter.Scale_Inverter;
    
    
        //----------trang thai ket noi --------------------
    pData[length++] = OBIS_ENVI_CONNECT_STATUS; 
    pData[length++] = 0x01;
    pData[length++] = sParam_IVT.Connect & 0xFF;
    
//     pData[length++] = OBIS_ENVI_ID_SERVER; 
//    pData[length++] = 0x04;
//    pData[length++] = (ID_server_u32 >> 24) & 0xFF;
//    pData[length++] = (ID_server_u32 >> 16) & 0xFF;
//    pData[length++] = (ID_server_u32 >> 8) & 0xFF;
//    pData[length++] = ID_server_u32 & 0xFF;
//    pData[length++] = 0x00;
       
//    if (sTempHumi.Status_u8[0] == true)
//    {        
//        //----------Nhiet do--------------------
//        pData[length++] = OBIS_ENVI_TEMP_1; // Dien ap pin   
//        pData[length++] = 0x02;
//        pData[length++] = (sTempHumi.sTemperature[0].Val_i16)>>8;
//        pData[length++] = (sTempHumi.sTemperature[0].Val_i16);
//        pData[length++] = sTempHumi.sTemperature[0].Scale_u8;
//        
//        //----------do am--------------------
//        pData[length++] = OBIS_ENVI_HUMI_1; // Dien ap pin
//        pData[length++] = 0x02;
//        pData[length++] = (sTempHumi.sHumidity[0].Val_i16)>>8;
//        pData[length++] = (sTempHumi.sHumidity[0].Val_i16);
//        pData[length++] = sTempHumi.sHumidity[0].Scale_u8;
//    }
//        
//    if (sTempHumi.Status_u8[1] == true)
//    {
//        //----------Nhiet do--------------------
//        pData[length++] = OBIS_ENVI_TEMP_2; // Dien ap pin   
//        pData[length++] = 0x02;
//        pData[length++] = (sTempHumi.sTemperature[1].Val_i16)>>8;
//        pData[length++] = (sTempHumi.sTemperature[1].Val_i16);
//        pData[length++] = sTempHumi.sTemperature[1].Scale_u8;
//        
//        //----------do am--------------------
//        pData[length++] = OBIS_ENVI_HUMI_2; // Dien ap pin
//        pData[length++] = 0x02;
//        pData[length++] = (sTempHumi.sHumidity[1].Val_i16)>>8;
//        pData[length++] = (sTempHumi.sHumidity[1].Val_i16);
//        pData[length++] = sTempHumi.sHumidity[1].Scale_u8;
//    }
//    
//    //----------Dong dien --------------------
//    pData[length++] = OBIS_EMET_CUR; // Dien ap pin
//    pData[length++] = 0x02;
//    pData[length++] = (sTempHumi.sCurrent.Val_i16)>>8;
//    pData[length++] = (sTempHumi.sCurrent.Val_i16);
//    pData[length++] = 0xFD;
//    
//    //----------Toa do GPS --------------------
//#ifdef USING_APP_SIM
////    if (sSimCommon.sGPS.Status_u8 == true)
////    {
//        if ( (sSimCommon.sGPS.LengData_u8 != 0) && (sSimCommon.sGPS.LengData_u8 < 30) )
//        {
//            pData[length++] = OBIS_GPS_LOC;  
//            pData[length++] = sSimCommon.sGPS.LengData_u8;
//            
//            for (i = 0; i < sSimCommon.sGPS.LengData_u8; i++)
//                pData[length++] = sSimCommon.sGPS.aPOS_INFOR[i]; 
//        }
////    }
//#endif
//
//    //----------Dien ap Pin--------------------
//    pData[length++] = OBIS_DEV_VOL1; // Dien ap pin
//    pData[length++] = 0x02;
//    pData[length++] = (sBattery.mVol_u32)>>8;
//    pData[length++] = (sBattery.mVol_u32);
//    pData[length++] = 0xFD;
//    
//    //----------Cuong do song--------------------
//#ifdef USING_APP_SIM
//    pData[length++] = OBIS_RSSI_1; 
//    pData[length++] = 0x01;
//    pData[length++] = sSimInfor.RSSI_u8;
//    pData[length++] = 0x00;   
//#endif
//    //----------Dien ap Pin--------------------
//    pData[length++] = OBIS_DEV_VOL2; // Dien ap pin
//    pData[length++] = 0x02;
//    pData[length++] = (sVout.mVol_u32)>>8;
//    pData[length++] = (sVout.mVol_u32);
//    pData[length++] = 0xFD;
//    
    //----------Tan suat--------------------
    pData[length++] = OBIS_FREQ_SEND; 
    pData[length++] = 0x02;
    pData[length++] = (sFreqInfor.FreqSendUnitMin_u32 >> 8) & 0xFF;
    pData[length++] = sFreqInfor.FreqSendUnitMin_u32 & 0xFF;
    pData[length++] = 0x00;

    // caculator crc
    length++;
	for (i = 0; i < (length - 1); i++)
		TempCrc ^= pData[i];

    pData[length-1] = TempCrc;
    
    
    return length;
}


uint8_t Modem_Packet_Event (uint8_t *pData)
{
    uint8_t     length = 0;
    uint16_t	i = 0;
    uint8_t     TempCrc = 0;
    
    //----------------------- start send data below ---------------------
    pData[length++] = 0x01;   // sTime
    pData[length++] = 0x06;
    pData[length++] = sRTC.year;
    pData[length++] = sRTC.month;
    pData[length++] = sRTC.date;
    pData[length++] = sRTC.hour;
    pData[length++] = sRTC.min;
    pData[length++] = sRTC.sec;
    
    //Chac chan vao day da co 1 alarm
    for (i = 0; i < _ALARM_END;i++)
    {
        if (aMARK_ALARM[i] == 1)
        {
            //Obis
            pData[length++] = i;
            //Length +Data + scale
            switch (i)
            {
                case _ALARM_FLOW_LOW:
                case _ALARM_FLOW_HIGH:
                    break;
                case _ALARM_PEAK_LOW:
                case _ALARM_PEAK_HIGH:
                    break;
                case _ALARM_VBAT_LOW:
                    break;
                case _ALARM_DIR_CHANGE:
                    break;
                case _ALARM_DETECT_CUT:
                    pData[length++] = 1;     //length
                    pData[length++] = sModem.DetectCutStatus_u8; 
                    pData[length++] = 0x00;
                    break;
                default:
                    break;   
            }
        }
        
        //Reset Alarm
        aMARK_ALARM[i] = 0;
    }  
            
    // caculator crc
    length++;
	for (i = 0; i < (length - 1); i++)
		TempCrc ^= pData[i];

    pData[length-1] = TempCrc;
    
    return length;
}


int32_t Modem_Cacul_Quantity (uint32_t PulseCur, uint32_t PulseOld)
{
    int32_t Qual = 0;
    
    if (PulseCur >=  PulseOld)
	{
		if ((PulseCur - PulseOld) < 0xFFFFFFAF)   //khong the quay nguoc dc 0x50 xung trong 10p va khong the quay tien 0xFFAF
			Qual = (PulseCur - PulseOld);  //binh thuong
		else
			Qual = - (0xFFFFFFFF - PulseCur + PulseOld + 1); //quay nguoc
	} else
	{
		if ((PulseOld - PulseCur) > 0x50)   //khong the quay nguoc dc 0x50 xung trong 10p
			Qual = (0xFFFFFFFF - PulseOld + PulseCur + 1);  //binh thuong
        else
			Qual = - (PulseOld - PulseCur);  // quay ngu?c
	}
    
    return Qual;
}





/*
    Func: Log Data Event
        + Packet Event Data
        + Save to Flash or ExMem
*/

void AppTemH_Log_Data_Event (void)
{
    uint8_t     aMessData[64] = {0};
    uint8_t     Length = 0;
    
    if (sRTC.year <= 20)
        return;
    
    Length = Modem_Packet_Event (&aMessData[0]);
    AppMem_Push_Mess_To_Queue_Write(_FLASH_TYPE_DATA_EVENT_A, &aMessData[0], Length);
}

/*
    Func: Log Data TSVH
        + Packet Data TSVH
        + Save to Flash or ExMem
*/

void AppIVT_Log_Data_TSVH (void)
{
    uint8_t     aMessData[256] = {0};
    uint8_t     Length = 0;
    
//    if (sRTC.year <= 20)
//        return;
    
    Length = AppIVT_Packet_TSVH (&aMessData[0]);
    AppMem_Push_Mess_To_Queue_Write(_FLASH_TYPE_DATA_TSVH_A, &aMessData[0], Length);
}

void AppIVT_Init (void)
{
//  OnchipFlashPageErase(ADDR_RUN_STOP_INVERTER);
//  OnchipFlashPageErase(ADDR_CONFIG_INVERTER);
//  OnchipFlashPageErase(ADDR_DEFAULT_INVERTER);
    Setup_Addr_Register_Default_IVT();
    Setup_Addr_Register_Write_IVT();
    Setup_Addr_Register_Run_Stop_IVT();
    
    
    
    //AppIVT_Init_Thresh_Measure();
    //AppIVT_Init_Slave_ID();
    //On power to 485 and tempetature 
    V12_IN_ON;
}

void Send_Command_IVT (uint8_t SlaveID, uint8_t Func_Code, uint16_t Addr_Register, uint16_t Infor_Register, void (*pFuncResetRecvData) (void)) 
{
  if(SlaveID >= 1 && SlaveID <= 127)
  {
    uint8_t aFrame[48] = {0};
    sData   strFrame = {(uint8_t *) &aFrame[0], 0};
    
    ModRTU_Master_Read_Frame(&strFrame, SlaveID, Func_Code, Addr_Register, Infor_Register);

    HAL_GPIO_WritePin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin, GPIO_PIN_SET);  
    HAL_Delay(10);
    // Send
    pFuncResetRecvData();
    
    HAL_UART_Transmit(&UART_485, strFrame.Data_a8, strFrame.Length_u16, 1000); 
    
    //Dua DE ve Receive
    HAL_GPIO_WritePin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin, GPIO_PIN_RESET);
  }
}

void AppIVT_Clear_Before_Recv (void)
{
    Reset_Buff(&sUart485);
}

/*
    Func: Check thu thap Temh co day du k?
        + Neu qua 10 lan: Reset cac node temp humi: Poweroff -> On again
*/

uint8_t AppTemh_Check_Status_TempH (void)
{
    uint8_t i = 0;
    uint8_t Result = true;
    
    //Check status
    for ( i = 0; i < sTempHumi.NumSlave_u8; i++)
    {
        if (sTempHumi.Status_u8[i] == false)
            break;         
    }
    //Neu tat ca deu true: i = sTempHumi.NumSlave_u8
    if (i == sTempHumi.NumSlave_u8)
    {
        //tat ca status deu true:
        sTempHumi.CountFailGetTemh_u8 = 0;
    } else
    {
        sTempHumi.CountFailGetTemh_u8++;
        if (sTempHumi.CountFailGetTemh_u8 >= MAX_GET_TEMH_FAIL)
        {
            sTempHumi.CountFailGetTemh_u8 = 0;

            Result = false;
        }
    }
    
    return Result;
}




void LED_Toggle (Led_TypeDef Led)
{
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}


void LED_On (Led_TypeDef Led)
{
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}


void LED_Off (Led_TypeDef Led)
{
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

static uint8_t _Cb_Temh_Log_TSVH (uint8_t event)
{
	//Log Data meter to flash
    UTIL_Printf_Str(DBLEVEL_M, "u_app_temh: packet tsvh!\r\n" );
    //
    Get_RTC();
         
//    AppTemH_Get_VBAT_mV();
//    UTIL_Printf_Str(DBLEVEL_M, "u_app_temh: dien ap pin trong: " );
//    UTIL_Printf_Dec(DBLEVEL_M, sBattery.mVol_u32);
//    UTIL_Printf_Str(DBLEVEL_M, " mV\r\n" );
//    
//    AppTemH_Get_Vout_mV();
//    //Print to debug
//    UTIL_Printf_Str(DBLEVEL_M, "u_app_temh: dien ap pin ngoai: " );
//    UTIL_Printf_Dec(DBLEVEL_M, sVout.mVol_u32);
//    UTIL_Printf_Str(DBLEVEL_M, " mV\r\n" );
// 
//    //Record TSVH
//    
//    //Cat chuoi data GPS luu lai: gui di
//    if (sAppSimVar.sDataGPS.Length_u16 > MAX_BYTE_CUT_GPS)
//    {
//        AppMem_Push_Mess_To_Queue_Write(_FLASH_TYPE_DATA_GPS_A, sAppSimVar.sDataGPS.Data_a8, sAppSimVar.sDataGPS.Length_u16);
//        sAppSimVar.sDataGPS.Length_u16 = 0;
//    }
    
    AppIVT_Log_Data_TSVH();    
        
    return 1;
}

